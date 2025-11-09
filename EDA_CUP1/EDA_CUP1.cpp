// File: main.cpp
// EDACup 2025 - Final version with full active-state detection

#include <exception>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <vector>
#include <CXXGraph/CXXGraph.hpp>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#define DEG_TO_RAD (3.1415926535F / 180.0F)

using namespace std;
using json = nlohmann::json;

// ====================================================
//  CONFIGURACIÓN DE CAMPO Y CONSTANTES
// ====================================================
const double HOME_GOAL_X = -1.0;
const double RIVAL_GOAL_X = 1.0;
const double CENTER_X = 0.0;
const double FIELD_Z_MIN = -0.8;
const double FIELD_Z_MAX = 0.8;
const double BALL_CONTROL_DIST = 0.07; // ~7 cm

// Conversión física
const double FIELD_LENGTH_CM = 219.0;
const double CM_PER_UNIT = FIELD_LENGTH_CM / 2.0; // 109.5 cm por unidad

// Área penal y margen
const double AREA_DEPTH_CM = 25.0;
const double AREA_HALF_WIDTH_CM = 40.0;
const double SAFETY_MARGIN_CM = 2.0;

const double AREA_DEPTH_U = AREA_DEPTH_CM / CM_PER_UNIT;       // ~0.23
const double AREA_HALF_WIDTH_U = AREA_HALF_WIDTH_CM / CM_PER_UNIT; // ~0.37
const double SAFETY_MARGIN_U = SAFETY_MARGIN_CM / CM_PER_UNIT; // ~0.018

// ====================================================
//  ESTRUCTURAS Y ESTADO DEL JUEGO
// ====================================================
struct Pos {
    double x, z;
    double distTo(const Pos& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (z - other.z) * (z - other.z));
    }
    double angleTo(const Pos& other) const {
        return std::atan2(other.z - z, other.x - x);
    }
};

struct GameState {
    Pos homeBot1, homeBot2;
    Pos rivalBot1, rivalBot2;
    Pos ball;
    bool playing = false;
    int frameCount = 0;

    // Altura y estado de los bots
    double homeBot1Height = 0.75;
    double homeBot2Height = 0.75;
    double rivalBot1Height = 0.75;
    double rivalBot2Height = 0.75;

    bool homeBot1Active = true;
    bool homeBot2Active = true;
    bool rivalBot1Active = true;
    bool rivalBot2Active = true;
};

// ====================================================
//  CLASE PRINCIPAL
// ====================================================
class SimpleRobot {
private:
    GameState state;

    // Detección de robot activo según altura (Y)
    bool isRobotActive(const json& botData) const {
        if (!botData.contains("position") || botData["position"].size() < 2) return false;
        return botData["position"][1] >= 0.75;
    }

    // Control de pelota
    bool hasControl(const Pos& robot, const Pos& ball) const {
        return robot.distTo(ball) < BALL_CONTROL_DIST;
    }

    // Enemigo más cercano a la pelota (solo activos)
    Pos getClosestEnemyToBall() const {
        vector<pair<Pos, bool>> rivals = {
            { state.rivalBot1, state.rivalBot1Active },
            { state.rivalBot2, state.rivalBot2Active }
        };

        double minDist = 999.0;
        Pos best = { CENTER_X, 0.0 };

        for (auto& [r, active] : rivals) {
            if (!active) continue;
            double d = r.distTo(state.ball);
            if (d < minDist) {
                minDist = d;
                best = r;
            }
        }
        return best;
    }

    bool enemiesInOurHalf() const {
        bool r1 = state.rivalBot1Active && state.rivalBot1.x < CENTER_X;
        bool r2 = state.rivalBot2Active && state.rivalBot2.x < CENTER_X;
        return r1 || r2;
    }

    void printPos(const Pos& p1, const Pos& p2) const {
        cerr << "[DEBUG] Rival1(" << p1.x << "," << p1.z << ") | Rival2(" << p2.x << "," << p2.z << ")" << endl;
    }

    // ====================================================
    //  FUNCIÓN AUXILIAR: Asegura que los robots no entren al área
    // ====================================================
    void clampPositionToRules(double& x, double& z, bool isHomeSide) {
        x = std::max(HOME_GOAL_X, std::min(RIVAL_GOAL_X, x));
        z = std::max(FIELD_Z_MIN, std::min(FIELD_Z_MAX, z));

        double depth = AREA_DEPTH_U + SAFETY_MARGIN_U;
        double halfZ = AREA_HALF_WIDTH_U - SAFETY_MARGIN_U;

        if (isHomeSide) {
            double limitX = HOME_GOAL_X + depth;
            if (x < limitX && std::abs(z) < halfZ)
                x = limitX;
        }
        else {
            double limitX = RIVAL_GOAL_X - depth;
            if (x > limitX && std::abs(z) < halfZ)
                x = limitX;
        }
    }

    // ====================================================
    //  COMPORTAMIENTO DEL ATACANTE
    // ====================================================
    json attackerBehavior(const Pos& robot) {
        json cmd;
        double distToBall = robot.distTo(state.ball);
        double angleToBall = robot.angleTo(state.ball);
        bool controlling = hasControl(robot, state.ball);

        if (!controlling) {
            double step = std::clamp(distToBall * 0.1, 0.03, 0.1);
            double nextX = robot.x + (state.ball.x - robot.x) * step;
            double nextZ = robot.z + (state.ball.z - robot.z) * step;

            clampPositionToRules(nextX, nextZ, false);

            cmd["positionXZ"] = { nextX, nextZ };
            cmd["rotationY"] = angleToBall;
            cmd["dribbler"] = (distToBall < 0.3) ? 1.0 : 0.5;
            cmd["kick"] = 0.0;
            cmd["chirp"] = 0.0;
        }
        else {
            Pos goal = { RIVAL_GOAL_X, 0.0 };
            double angleToGoal = robot.angleTo(goal);
            double distToGoal = robot.distTo(goal);

            double nextX = robot.x + (goal.x - robot.x) * 0.1;
            double nextZ = robot.z + (goal.z - robot.z) * 0.1;

            clampPositionToRules(nextX, nextZ, false);

            cmd["positionXZ"] = { nextX, nextZ };
            cmd["rotationY"] = angleToGoal;
            cmd["dribbler"] = 1.0;
            cmd["kick"] = (distToGoal < 0.3 && state.ball.x > 0.4) ? 1.0 : 0.0;
            cmd["chirp"] = 0.0;
        }

        return cmd;
    }

    // ====================================================
    //  COMPORTAMIENTO DEL DEFENSOR
    // ====================================================
    json defenderBehavior(const Pos& robot) {
        json cmd;
        if (!enemiesInOurHalf()) {
            double patrolX = HOME_GOAL_X + 0.35;
            double patrolZ = 0.3 * std::sin(state.frameCount * 0.05);

            clampPositionToRules(patrolX, patrolZ, true);

            cmd["positionXZ"] = { patrolX, patrolZ };
            cmd["rotationY"] = robot.angleTo(state.ball);
            cmd["dribbler"] = 0.0;
            cmd["kick"] = 0.0;
            cmd["chirp"] = 0.0;
        }
        else {
            Pos target = getClosestEnemyToBall();

            double interceptX = HOME_GOAL_X + (target.x - HOME_GOAL_X) * 0.6;
            double interceptZ = target.z * 0.6;

            clampPositionToRules(interceptX, interceptZ, true);

            cmd["positionXZ"] = { interceptX, interceptZ };
            cmd["rotationY"] = robot.angleTo(state.ball);
            cmd["dribbler"] = 0.4;
            cmd["kick"] = 0.0;
            cmd["chirp"] = 0.0;
        }

        return cmd;
    }

public:
    // ====================================================
    //  ACTUALIZAR ESTADO DEL JUEGO
    // ====================================================
    void updateState(const json& data) {
        // Home bots
        state.homeBot1Active = isRobotActive(data["homeBot1"]);
        state.homeBot2Active = isRobotActive(data["homeBot2"]);

        if (state.homeBot1Active)
            state.homeBot1 = { data["homeBot1"]["position"][0], data["homeBot1"]["position"][2] };
        if (state.homeBot2Active)
            state.homeBot2 = { data["homeBot2"]["position"][0], data["homeBot2"]["position"][2] };

        // Rivales
        state.rivalBot1Active = isRobotActive(data["rivalBot1"]);
        state.rivalBot2Active = isRobotActive(data["rivalBot2"]);

        if (state.rivalBot1Active)
            state.rivalBot1 = { data["rivalBot1"]["position"][0], data["rivalBot1"]["position"][2] };
        if (state.rivalBot2Active)
            state.rivalBot2 = { data["rivalBot2"]["position"][0], data["rivalBot2"]["position"][2] };

        // Pelota
        state.ball = { data["ball"]["position"][0], data["ball"]["position"][2] };
        state.frameCount++;
    }

    void setPlaying(bool playing) { state.playing = playing; }

    // ====================================================
    //  DECISIÓN PRINCIPAL
    // ====================================================
    json makeDecision() {
        json response;
        response["type"] = "set";

        if (!state.playing) {
            response["data"]["homeBot1"] = { {"positionXZ", {state.homeBot1.x, state.homeBot1.z}},
                                             {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0} };
            response["data"]["homeBot2"] = { {"positionXZ", {state.homeBot2.x, state.homeBot2.z}},
                                             {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0} };
            return response;
        }

        // Bots activos
        if (state.homeBot1Active)
            response["data"]["homeBot1"] = attackerBehavior(state.homeBot1);
        else
            response["data"]["homeBot1"] = { {"positionXZ", {state.homeBot1.x, state.homeBot1.z}},
                                             {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0} };

        if (state.homeBot2Active)
            response["data"]["homeBot2"] = defenderBehavior(state.homeBot2);
        else
            response["data"]["homeBot2"] = { {"positionXZ", {state.homeBot2.x, state.homeBot2.z}},
                                             {"rotationY", 0}, {"dribbler", 0}, {"kick", 0}, {"chirp", 0} };

        printPos(state.rivalBot1, state.rivalBot2);
        return response;
    }
};

// ====================================================
//  MAIN LOOP
// ====================================================
int main(int argc, char* argv[]) {
    SimpleRobot robot;
    while (true) {
        try {
            string line;
            if (!getline(cin, line)) break;
            json message = json::parse(line);
            string type = message["type"];

            if (type == "start") robot.setPlaying(true);
            else if (type == "stop") robot.setPlaying(false);
            else if (type == "state") {
                robot.updateState(message["data"]);
                json response = robot.makeDecision();
                cout << response.dump() << endl;
            }
        }
        catch (exception& e) {
            cerr << "[EXCEPTION] " << e.what() << endl;
        }
    }
    return 0;
}
