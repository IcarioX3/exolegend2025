#include "gladiator.h"
#include <queue>
#include <unordered_map>

Gladiator *gladiator;
Maze *maze;

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;

struct Node {
    MazeSquare *square;
    Node *parent;
    float g, h;
    float f() const { return g + h; }
};

struct CompareNode {
    bool operator()(const Node *a, const Node *b) { return a->f() > b->f(); }
};

std::vector<MazeSquare *> reconstruct_path(Node *node) {
    std::vector<MazeSquare *> path;
    while (node) {
        path.push_back(node->square);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<MazeSquare *> a_star(MazeSquare *start, MazeSquare *goal) {
    std::priority_queue<Node *, std::vector<Node *>, CompareNode> openSet;
    std::unordered_map<MazeSquare *, Node *> allNodes;

    Node *startNode = new Node{start, nullptr, 0, 0};
    openSet.push(startNode);
    allNodes[start] = startNode;

    while (!openSet.empty()) {
        Node *current = openSet.top();
        openSet.pop();

        if (current->square == goal) return reconstruct_path(current);

        for (MazeSquare *neighbor : {current->square->northSquare, current->square->southSquare, 
                                     current->square->eastSquare, current->square->westSquare}) {
            if (!neighbor) continue;
            float g_score = current->g + 1;
            
            if (allNodes.find(neighbor) == allNodes.end() || g_score < allNodes[neighbor]->g) {
                Node *neighborNode = new Node{neighbor, current, g_score, 0};
                neighborNode->h = abs(neighbor->x - goal->x) + abs(neighbor->y - goal->y);
                openSet.push(neighborNode);
                allNodes[neighbor] = neighborNode;
            }
        }
    }
    return {};
}

MazeSquare *find_nearest_bomb(MazeSquare *start) {
    std::queue<MazeSquare *> q;
    std::unordered_map<MazeSquare *, bool> visited;
    q.push(start);
    visited[start] = true;
    
    while (!q.empty()) {
        MazeSquare *current = q.front();
        q.pop();
        
        if (current->coin == 1) return current;
        
        for (MazeSquare *neighbor : {current->northSquare, current->southSquare, 
                                     current->eastSquare, current->westSquare}) {
            if (neighbor && !visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    return nullptr;
}

void go_to(Position cons, Position pos) {
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos) {
        double rho = atan2(dy, dx);
        double consw = kw * (rho - pos.a);
        double consv = kv * d * cos(rho - pos.a);
        consw = abs(consw) > wlimit ? (consw > 0 ? wlimit : -wlimit) : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? vlimit : -vlimit) : consv;
        
        consvl = consv - gladiator->robot->getRobotRadius() * consw;
        consvr = consv + gladiator->robot->getRobotRadius() * consw;
    } else {
        consvr = 0;
        consvl = 0;
    }
    
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
}

void reset() {
    gladiator->log("Call of reset function");
}

void setup() {
    gladiator = new Gladiator();
    maze = new Maze();
    gladiator->game->onReset(&reset);
}

void loop() {
    if (gladiator->game->isStarted()) {
        MazeSquare *currentSquare = maze->getSquare(gladiator->robot->getData().position);
        MazeSquare *targetSquare = find_nearest_bomb(currentSquare);
        
        if (targetSquare) {
            std::vector<MazeSquare *> path = a_star(currentSquare, targetSquare);
            if (!path.empty()) {
                Position nextPos = {path[1]->x, path[1]->y, 0};
                go_to(nextPos, gladiator->robot->getData().position);
            }
        }
    }
    delay(10);
}
