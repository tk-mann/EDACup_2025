// EDACup example

#include <exception>
#include <cmath>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include <vector>
#include <CXXGraph/CXXGraph.hpp>  // Librería instalada con vcpkg

#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#define DEG_TO_RAD (3.1415926535F / 180.0F)
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif




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
         }}},
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
const double BALL_CONTROL_DIST = 0.07;  // ~15cm en escala normalizada



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

// Normaliza un ángulo a [-pi, +pi]
inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Utilidad clamp para doubles
inline double clampd(double v, double a, double b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

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

        // Si ambos están en nuestro lado, elegir el más cercano
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

        // Si ninguno está en nuestro lado, devolver el más cercano
        return (dist1 < dist2) ? state.rivalBot1 : state.rivalBot2;
    }


    bool enemiesInOurHalf() const {
        return state.rivalBot1.x < CENTER_X || state.rivalBot2.x < CENTER_X;
    }

    void printPos(const Pos& pos1, const Pos& pos2) const {
        cerr << "Pos1: (" << pos1.x << ", " << pos1.z << ") | Pos2: (" << pos2.x << ", " << pos2.z << ")" << endl;
    }

    double computeAngle(const Pos& from, const Pos& to) {
        double dx = to.x - from.x;
        double dz = to.z - from.z;

        // Ángulo en radianes usando la convención del simulador:
        //   forward = +Z
        //   angle = atan2(dx, dz)
        return std::atan2(dx, dz) + 3.1415926535F;
    }
    json attackerBehavior(const Pos& robot, bool isBot1) {
        json command;
        Pos ball = state.ball;
        Pos goal = { RIVAL_GOAL_X, 0.0 };

        // Calcular distancia a la pelota
        double dx = robot.x - ball.x;
        double dz = robot.z - ball.z;
        double dist = std::sqrt(dx * dx + dz * dz);

        // LÍMITES DEL ÁREA RIVAL
        const double ATTACK_LIMIT_X = RIVAL_GOAL_X - 0.23;
        const double AREA_LIMIT_Z = 0.37;  // Ancho del área

        // FASE 1: Acercarse a la pelota
        if (dist > 0.12) {
            double angleToBall = computeAngle(robot, ball);
            command["rotationY"] = angleToBall;

            double targetX = ball.x;
            double targetZ = ball.z;

            // Si está dentro del ancho del área, limitar X
            if (std::abs(targetZ) < AREA_LIMIT_Z) {
                targetX = std::min(targetX, ATTACK_LIMIT_X);
            }

            command["positionXZ"] = { targetX, targetZ };
            command["dribbler"] = 1.0;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
            return command;
        }

        // FASE 2: Enganchar la pelota (rotar sin moverse)
        if (dist > 0.007) {
            double angleToBall = computeAngle(robot, ball);
            command["rotationY"] = angleToBall;

            double targetX = goal.x;
            double targetZ = goal.z;

            // Si está dentro del ancho del área, limitar X
            if (std::abs(targetZ) < AREA_LIMIT_Z) {
                targetX = std::min(targetX, ATTACK_LIMIT_X);
            }

            command["positionXZ"] = { targetX, targetZ };
            command["dribbler"] = 1.0;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
        }

        // FASE 3: Tiene la pelota enganchada
        double angleToGoal = computeAngle(robot, goal);
        bool pastHalfField = (robot.x > 0.0);  // Pasó la mitad de cancha

        // Siempre rotar hacia el arco
        command["rotationY"] = angleToGoal;
        command["dribbler"] = 1.0;

        // Si pasó la mitad de cancha → DISPARAR
        if (pastHalfField) {
            command["positionXZ"] = { robot.x, robot.z };  // Quedarse quieto
            command["kick"] = 1.0;  // ¡DISPARAR!
            command["chirp"] = 0.0;
            cerr << "DISPARANDO desde x=" << robot.x << ", z=" << robot.z << endl;
        }
        else {
            // Aún no llegó a la mitad → seguir avanzando
            double targetX = goal.x;
            double targetZ = goal.z;

            // Si está dentro del ancho del área, limitar X
            if (std::abs(targetZ) < AREA_LIMIT_Z) {
                targetX = std::min(targetX, ATTACK_LIMIT_X);
            }

            command["positionXZ"] = { targetX, targetZ };
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
        }

        return command;
    }
    // Defender: se mantiene entre arco y pelota sin entrar al área, se mueve rápido y defiende activamente
    json defenderBehavior(const Pos& robot, bool isBot1) {
        json command;
        Pos goal = { HOME_GOAL_X, 0.0 };
        Pos ball = state.ball;
        double angleToBall = robot.angleTo(ball);
        double distToBall = robot.distTo(ball);

        // Posicionamiento más agresivo según cercanía de la pelota
        double ratio = 0.3;
        if (ball.x < CENTER_X) ratio = 0.5;
        if (ball.x < HOME_GOAL_X + 0.4) ratio = 0.65;  // Más agresivo cerca del arco

        double targetX = goal.x + (ball.x - goal.x) * ratio;
        double targetZ = goal.z + (ball.z - goal.z) * ratio;

        const double DEFENSE_LIMIT_X = HOME_GOAL_X + 0.23;
        const double DEFENSE_LIMIT_Z = 0.37;

        targetX = std::max(targetX, DEFENSE_LIMIT_X);
        targetZ = std::clamp(targetZ, -DEFENSE_LIMIT_Z, DEFENSE_LIMIT_Z);

        double alpha = 0.75;

        // Si la pelota está muy cerca, ir directo sin suavizado
        if (distToBall < 0.4) {
            alpha = 1.0;  // Movimiento instantáneo cuando está cerca
        }

        double smoothX = robot.x + (targetX - robot.x) * alpha;
        double smoothZ = robot.z + (targetZ - robot.z) * alpha;

        command["positionXZ"] = { smoothX, smoothZ };
        command["rotationY"] = angleToBall;

        // Defensa activa: intenta controlar la pelota cuando está cerca
        if (distToBall < 0.35) {  // Rango más amplio
            command["dribbler"] = 1.0;  // Dribbler más fuerte

            // Si tiene la pelota y está bien posicionado, despeja
            double angleToOwnGoal = robot.angleTo(goal);
            if (distToBall < 0.15 && std::abs(angleToBall - angleToOwnGoal) > 1.5) {
                command["kick"] = 1.0;  // Despeja con fuerza
            }
            else {
                command["kick"] = 0.0;
            }
            command["chirp"] = 0.3;
        }
        else {
            command["dribbler"] = 0.0;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;
        }

        return command;
    }
public:
    void updateState(const json& data) {
        // Actualizar posiciones
        state.homeBot1 = { data["homeBot1"]["position"][0], data["homeBot1"]["position"][2] };
        state.homeBot2 = { data["homeBot2"]["position"][0], data["homeBot2"]["position"][2] };
        state.rivalBot1 = { data["rivalBot1"]["position"][0], data["rivalBot1"]["position"][2] };
        state.rivalBot2 = { data["rivalBot2"]["position"][0], data["rivalBot2"]["position"][2] };
        state.ball = { data["ball"]["position"][0], data["ball"]["position"][2] };
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
                                  {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0}}},
                    {"homeBot2", {{"positionXZ", {state.homeBot2.x, state.homeBot2.z}},
                                  {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0}}}
                }}
            };
        }

        // ESTRATEGIA SIMPLE:
        // Bot1 = ATACANTE (busca pelota y ataca)
        // Bot2 = DEFENSOR (protege arco)

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
    bool isRunning = false;
    uint32_t time = 0;
    SimpleRobot robot;

    while (true)
    {
        // Try-catch allows intercepting errors.
        // On error, the catch block is executed.
        try
        {
            string line;
            getline(cin, line);
            json message = json::parse(line);

            string type = message["type"];
            if (type == "start") {
                robot.setPlaying(true);
            }
            else if (type == "stop") {
                robot.setPlaying(false);
            }
            else if (type == "state")
            {
                robot.updateState(message["data"]);
                json response = robot.makeDecision();
                //cerr << "Robots possitions - HomeBot1: (" << response["data"]["homeBot1"]["positionXZ"][0] << ", " << response["data"]["homeBot1"]["positionXZ"][1] << "), HomeBot2: (" << response["data"]["homeBot2"]["positionXZ"][0] << ", " << response["data"]["homeBot2"]["positionXZ"][0] << ")" << endl;
                std::cout << response.dump() << std::endl;
            }
        }
        catch (exception& error)
        {
            // cerr prints to debug console
            cerr << error.what() << endl;
        }
    }
}
