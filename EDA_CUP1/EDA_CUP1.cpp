// File: main.cpp
// EDACup example 

#include <exception>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>               // std::clamp
#include <nlohmann/json.hpp>
#include <vector>
#include <CXXGraph/CXXGraph.hpp>   // Librería instalada con vcpkg 
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#define DEG_TO_RAD (3.1415926535F / 180.0F)

using namespace std;
using json = nlohmann::json;

void poseHomeBot1(float positionX, float positionZ, float rotationY)
{
    json sampleMessage = {
        {"type", "set"},
        {"data",
         {{
             "homeBot1",
             {
                 {"positionXZ", {positionX, positionZ}},
                 {"rotationY", rotationY},
             },
         }}} ,
    };

    // cout connects to server
    cout << sampleMessage.dump() << endl;

    // cerr prints to debug console
    cerr << "Updated homeBot1 pose." << endl;
}

// Constantes del campo
const double HOME_GOAL_X = -1.0;
const double RIVAL_GOAL_X = 1.0;
const double CENTER_X = 0.0;
const double FIELD_Z_MIN = -0.8;
const double FIELD_Z_MAX = 0.8;
const double BALL_CONTROL_DIST = 0.07;  // ~7 cm (ajustable)

// Estructura para posición
struct Pos {
    double x, z;

    double distTo(const Pos& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (z - other.z) * (z - other.z));
    }

    double angleTo(const Pos& other) const {
        return std::atan2(other.z - z, other.x - x);
    }
};

// Estado del juego
struct GameState {
    Pos homeBot1, homeBot2;
    Pos rivalBot1, rivalBot2;
    Pos ball;
    bool playing = false;
    int frameCount = 0;
};

class SimpleRobot {
private:
    GameState state;

    // Verifica si un robot tiene control de la pelota
    bool hasControl(const Pos& robot, const Pos& ball) const {
        return robot.distTo(ball) < BALL_CONTROL_DIST;
    }

    Pos getClosestEnemyToBall() const {
        double dist1 = state.rivalBot1.distTo(state.ball);
        double dist2 = state.rivalBot2.distTo(state.ball);

        bool rival1InOurHalf = state.rivalBot1.x < CENTER_X;
        bool rival2InOurHalf = state.rivalBot2.x < CENTER_X;

        if (rival1InOurHalf && rival2InOurHalf) {
            return (dist1 < dist2) ? state.rivalBot1 : state.rivalBot2;
        }
        else if (rival1InOurHalf) {
            return state.rivalBot1;
        }
        else if (rival2InOurHalf) {
            return state.rivalBot2;
        }

        return (dist1 < dist2) ? state.rivalBot1 : state.rivalBot2;
    }

    bool enemiesInOurHalf() const {
        return state.rivalBot1.x < CENTER_X || state.rivalBot2.x < CENTER_X;
    }

    void printPos(const Pos& pos1, const Pos& pos2) const {
        cerr << "Pos1: (" << pos1.x << ", " << pos1.z << ") | Pos2: (" << pos2.x << ", " << pos2.z << ")" << endl;
    }

    // Robot atacante: busca pelota, la controla y dispara al arco
    json attackerBehavior(const Pos& robot, bool isBot1) {
        json command;

        // CÁLCULO INICIAL 
        double distToBall = robot.distTo(state.ball);
        double angleToBall = robot.angleTo(state.ball);
        bool controlling = hasControl(robot, state.ball);

        // COMPORTAMIENTO SIN PELOTA
        if (!controlling) {
            // angleDiff no necesita recalcular la misma función; esto es la diferencia
            // (si existiera orientación actual del robot, usarla; aquí asumimos que
            // rotationY de salida será la orientación deseada).
            double angleDiff = 0.0; // placeholder semántico (podrías usar orientación actual si la tuvieras)

            // Si está lejos y presumiblemente desalineado, prioriza girar sobre su eje
            // (usamos distToBall y un umbral angular implícito).
            if (distToBall > 0.2 && std::abs(angleDiff) > 0.3) {
                // Gira en el lugar (posición igual, solo rota hacia la pelota)
                command["positionXZ"] = { robot.x, robot.z };
                command["rotationY"] = angleToBall;
            }
            else {
                // Se mueve suavemente hacia la pelota, step escalado por distancia
                double step = std::clamp(distToBall * 0.1, 0.03, 0.1);
                command["positionXZ"] = {
                    robot.x + (state.ball.x - robot.x) * step,
                    robot.z + (state.ball.z - robot.z) * step
                };
                command["rotationY"] = angleToBall;
            }

            // Dribblea: más fuerte si está cerca
            command["dribbler"] = (distToBall < 0.3) ? 1.0 : 0.5;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
        }

        // COMPORTAMIENTO CON PELOTA
        else {
            Pos goal = { RIVAL_GOAL_X, 0.0 };
            double angleToGoal = robot.angleTo(goal);
            double distToGoal = robot.distTo(goal);
            double angleDiffToGoal = std::abs(angleToGoal - angleToBall);

            // Mantiene control total del dribbler mientras avanza
            command["dribbler"] = 1.0;
            command["rotationY"] = angleToGoal;

            // Movimiento hacia el arco
            double nextX = robot.x + (goal.x - robot.x) * 0.1;
            double nextZ = robot.z + (goal.z - robot.z) * 0.1;

            // Límite para no entrar totalmente al área rival (25 cm, 80 cm)
            const double RIVAL_AREA_X = RIVAL_GOAL_X - 0.23; // 0.23 unidades aproximadas
            const double RIVAL_AREA_Z = 0.37;

            if (nextX > RIVAL_AREA_X && std::abs(nextZ) < RIVAL_AREA_Z) {
                nextX = RIVAL_AREA_X; // detenerse justo fuera del área
            }

            command["positionXZ"] = { nextX, nextZ };
            command["chirp"] = 0.0;

            // Disparo controlado: usar distancia real al arco (no solo x)
            if (distToGoal < 0.3 && std::abs(angleDiffToGoal) < 0.25 && state.ball.x > 0.4) {
                command["kick"] = 1.0;
            }
            else {
                command["kick"] = 0.0;
            }
        }
        return command;
    }
    // Robot defensor : se mueve sobre el arco propio
    json defenderBehavior(const Pos& robot, bool isBot1) {
        json command;

        // Distancia a la pelota
        double distToBall = robot.distTo(state.ball);

        if (!enemiesInOurHalf()) {
            // Movimiento lateral controlado frente al arco, sin entrar al área
            double patrolZ = 0.3 * std::sin(state.frameCount * 0.05);  // barrido suave de -0.3 a 0.3
            double patrolX = HOME_GOAL_X + 0.35;                       // fija posición fuera del área

            command["positionXZ"] = { patrolX, patrolZ };
            command["rotationY"] = robot.angleTo(state.ball);          // mirar hacia la pelota
            command["dribbler"] = 0.0;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
        }
        else {
            // DEFENSA ACTIVA
            Pos target = getClosestEnemyToBall();

            // Límites del área de penal
            const double DEFENSE_LIMIT_X = HOME_GOAL_X + 0.23;
            const double DEFENSE_LIMIT_Z = 0.37;

            // Si el enemigo está dentro del área, lo “proyectamos” hacia fuera
            if (target.x < DEFENSE_LIMIT_X && std::abs(target.z) < DEFENSE_LIMIT_Z) {
                target.x = DEFENSE_LIMIT_X;
            }

            // Intercepto más adelantado (⅔ hacia el enemigo)
            double interceptX = HOME_GOAL_X + (target.x - HOME_GOAL_X) * 0.66;
            double interceptZ = target.z * 0.5;  // un poco más centrado

            // Limitar dentro del campo
            interceptX = std::max(HOME_GOAL_X, std::min(CENTER_X - 0.05, interceptX));
            interceptZ = std::max(FIELD_Z_MIN, std::min(FIELD_Z_MAX, interceptZ));

            // Orientar hacia la pelota (mejor referencia que el enemigo)
            double angleToBall = robot.angleTo(state.ball);

            command["positionXZ"] = { interceptX, interceptZ };
            command["rotationY"] = angleToBall;

            // Dribbler solo si la pelota se acerca (para “bloquear”)
            command["dribbler"] = (distToBall < 0.25) ? 0.6 : 0.0;
            command["kick"] = 0.0;
            command["chirp"] = (distToBall < 0.25) ? 0.2 : 0.0; // aviso sutil
        }

        return command;
    }

public:
    void updateState(const json& data) {
        // Actualizar posiciones según formato del simulador (x,y,z) - usamos x,z
        state.homeBot1 = { data["homeBot1"]["position"][0], data["homeBot1"]["position"][2] };
        state.homeBot2 = { data["homeBot2"]["position"][0], data["homeBot2"]["position"][2] };
        state.rivalBot1 = { data["rivalBot1"]["position"][0], data["rivalBot1"]["position"][2] };
        state.rivalBot2 = { data["rivalBot2"]["position"][0], data["rivalBot2"]["position"][2] };
        state.ball = { data["ball"]["position"][0],     data["ball"]["position"][2] };
        state.frameCount++;
    }

    void setPlaying(bool playing) {
        state.playing = playing;
    }

    json makeDecision() {
        if (!state.playing) {
            // Si no está jugando, mantener robots quietos
            return {
                {"type", "set"},
                {"data", {
                    {"homeBot1", {{"positionXZ", {state.homeBot1.x, state.homeBot1.z}},
                                  {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0}} },
                    {"homeBot2", {{"positionXZ", {state.homeBot2.x, state.homeBot2.z}},
                                  {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0}} }
                }}
            };
        }

        json response;
        response["type"] = "set";
        response["data"]["homeBot1"] = attackerBehavior(state.homeBot1, true);
        response["data"]["homeBot2"] = defenderBehavior(state.homeBot2, false);

        printPos(state.rivalBot1, state.rivalBot2);

        return response;
    }
};


int main(int argc, char* argv[])
{
    SimpleRobot robot;

    while (true)
    {
        try
        {
            string line;
            if (!getline(cin, line)) {
                // EOF o pipe cerrado
                break;
            }
            json message = json::parse(line);
            string type = message["type"];
            if (type == "start") {
                robot.setPlaying(true);
            }
            else if (type == "stop") {
                robot.setPlaying(false);
            }
            else if (type == "state") {
                robot.updateState(message["data"]);
                json response = robot.makeDecision();
                cout << response.dump() << std::endl;
            }
        }
        catch (exception& error)
        {
            cerr << "EXCEPTION: " << error.what() << endl;
        }
    }

    return 0;
}





