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

    void printPos(const Pos& pos1, const Pos& pos2 ) const {
		cerr << "Pos1: (" << pos1.x << ", " << pos1.z << ") | Pos2: (" << pos2.x << ", " << pos2.z << ")" << endl;      
    }

    // Robot atacante: busca pelota, la controla y dispara al arco
    json attackerBehavior(const Pos& robot, bool isBot1) {
        json command;

        bool controlling = hasControl(robot, state.ball);

        if (!controlling) {
            // BUSCAR LA PELOTA: ir directo hacia ella
            command["positionXZ"] = { state.ball.x, state.ball.z };
            command["rotationY"] = robot.angleTo(state.ball);
            command["dribbler"] = 1.0;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;

        }
        else {
            // TIENE LA PELOTA: ir hacia el arco rival (x = 1.0, z = 0)
            Pos goal = { RIVAL_GOAL_X, 0.0 };
            double distToGoal = state.ball.distTo(goal);

            // Avanzar hacia el centro del arco rival
            command["positionXZ"] = { goal.x, goal.z };
            command["rotationY"] = robot.angleTo(goal);
            command["dribbler"] = 1.0;

            // DISPARAR cuando está en buen rango (< 0.3 en escala normalizada, ~65cm)
            if (distToGoal < 0.35 && state.ball.x > 0.5) {  // Debe estar en campo rival
                double angleToGoal = state.ball.angleTo(goal);
                double currentAngle = robot.angleTo(goal);
                double angleDiff = std::abs(angleToGoal - currentAngle);

                // Si está bien alineado, PATEAR
                if (angleDiff < 0.4) {  // ~23 grados de tolerancia
                    command["kick"] = 1.0;
                }
                else {
                    command["kick"] = 0.0;
                }
            }
            else {
                command["kick"] = 0.0;
            }
            command["chirp"] = 0.0;
        }

        return command;
    }

    // Robot defensor: se mueve sobre el arco propio de forma errática
    json defenderBehavior(const Pos& robot, bool isBot1) {
        json command;

        if (!enemiesInOurHalf()) {
            // NO HAY ENEMIGOS: patrullar erráticamente por nuestro lado
            // Movimiento aleatorio usando funciones trigonométricas
            double targetX = HOME_GOAL_X + 0.5 * (1.0 + std::sin(state.frameCount * 0.05));  // Entre -1.0 y -0.5
            double targetZ = 0.6 * std::sin(state.frameCount * 0.08 + 1.5);  // Entre -0.6 y 0.6

            command["positionXZ"] = { targetX, targetZ };
            command["rotationY"] = robot.angleTo(state.ball);
            command["dribbler"] = 0.2;
            command["kick"] = 0.0;
            command["chirp"] = 0.0;

        }
        else {
            // HAY ENEMIGOS: marcar al más cercano a la pelota
            Pos target = getClosestEnemyToBall();

            // Posicionarse entre el enemigo y nuestro arco
            double interceptX = (target.x + HOME_GOAL_X) / 2.0;  // Punto medio
            double interceptZ = target.z * 0.8;  // Ligeramente hacia el centro

            // Limitar a nuestro lado
            interceptX = std::max(HOME_GOAL_X, std::min(CENTER_X - 0.05, interceptX));
            interceptZ = std::max(FIELD_Z_MIN, std::min(FIELD_Z_MAX, interceptZ));

            command["positionXZ"] = { interceptX, interceptZ };
            command["rotationY"] = robot.angleTo(target);
            command["dribbler"] = 0.5;
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




