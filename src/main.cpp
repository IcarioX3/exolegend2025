#include "gladiator.h"
#include <cmath>
#include <queue>
#undef abs

Gladiator *gladiator;
Maze *maze;

int MAZE_SIZE = 3;
float CELL_SIZE = MAZE_SIZE / 12.f;

float kw = 1.7;
float kv = 1.5;
float wlimit = 3.f;
float vlimit = 2.5f;
float erreurPos = 0.5;

int MAZE_TRACK[12][12] = {0};

// unsigned long previousTime = 0; // Stocke le dernier moment où la variable a été incrémentée
// const unsigned long interval = 13000; // Intervalle de 14 secondes (en millisecondes)
// int counter = 0; // Variable à incrémenter

class Vector2
{
  public:
    Vector2() : _x(0.), _y(0.)
    {
    }
    Vector2(float x, float y) : _x(x), _y(y)
    {
    }

    float norm1() const
    {
        return std::abs(_x) + std::abs(_y);
    }
    float norm2() const
    {
        return std::sqrt(_x * _x + _y * _y);
    }
    void normalize()
    {
        _x /= norm2();
        _y /= norm2();
    }
    Vector2 normalized() const
    {
        Vector2 out = *this;
        out.normalize();
        return out;
    }

    Vector2 operator-(const Vector2 &other) const
    {
        return {_x - other._x, _y - other._y};
    }
    Vector2 operator+(const Vector2 &other) const
    {
        return {_x + other._x, _y + other._y};
    }
    Vector2 operator*(float f) const
    {
        return {_x * f, _y * f};
    }

    bool operator==(const Vector2 &other) const
    {
        return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5;
    }
    bool operator!=(const Vector2 &other) const
    {
        return !(*this == other);
    }

    float x() const
    {
        return _x;
    }
    float y() const
    {
        return _y;
    }

    float dot(const Vector2 &other) const
    {
        return _x * other._x + _y * other._y;
    }
    float cross(const Vector2 &other) const
    {
        return _x * other._y - _y * other._x;
    }
    float angle(const Vector2 &m) const
    {
        return std::atan2(cross(m), dot(m));
    }
    float angle() const
    {
        return std::atan2(_y, _x);
    }

  private:
    float _x, _y;
};

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

void go_to(Vector2 cons, Vector2 pos)
{
    double consvl, consvr;
    Vector2 delta = cons - pos;
    double d = delta.norm2();

    double rho = atan2(delta.y(), delta.x());
    double consw = kw * reductionAngle(rho - gladiator->robot->getData().position.a);

    double consv = kv * d * cos(reductionAngle(rho - gladiator->robot->getData().position.a));
    consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
    consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

    consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
    consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 12; j++)
        {
            MAZE_TRACK[i][j] = 0;
        }
    }
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    maze = new Maze();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
    Serial.begin(9600); // Initialiser la communication série
}

Vector2 index_to_coordinates(int i, int j) {
    return Vector2(i * CELL_SIZE + CELL_SIZE / 2, j * CELL_SIZE + CELL_SIZE / 2);
}

Vector2 coordinates_to_index(Vector2 pos) {
    return Vector2((int)(pos.x() / CELL_SIZE), (int)(pos.y() / CELL_SIZE));
}

void update_maze() {
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posIndex = coordinates_to_index(pos);
    Vector2 target = index_to_coordinates(posIndex.x(), posIndex.y());
    if ((pos - target).norm2() < erreurPos) {
        MAZE_TRACK[(int)posIndex.x()][(int)posIndex.y()] = 1;
    }
}

struct Node {
    int x, y;
    Node() : x(0), y(0) {}
    Node(int x, int y) : x(x), y(y) {}
    bool operator==(const Node& other) const { return x == other.x && y == other.y; }
};

Node bfs_find_bomb(Vector2 start) {
    Node queue[144]; // 12 * 12 maximum nodes
    bool visited[12][12] = {false};
    Node parent[12][12]; // Tableau pour stocker le parent de chaque case

    int front = 0, back = 0;
    queue[back++] = Node(start.x(), start.y());
    visited[(int)start.x()][(int)start.y()] = true;
    parent[(int)start.x()][(int)start.y()] = Node(-1, -1); // Marqueur de début

    while (front < back) {
        Node current = queue[front++];
        MazeSquare *sq = maze->getSquare(current.x, current.y);
        
        // Si une bombe est trouvée, reconstruire le chemin
        if (sq && sq->coin.value > 0) {
            Node step = current;
            while (!(parent[step.x][step.y] == Node(-1, -1))) { // Tant qu'on n'est pas à l'origine
                if (parent[parent[step.x][step.y].x][parent[step.x][step.y].y] == Node(-1, -1)) {
                    return step; // Premier mouvement depuis le départ
                }
                step = parent[step.x][step.y];
            }
        }

        Node neighbors[4];
        int neighborCount = 0;
        if (sq->northSquare != nullptr) neighbors[neighborCount++] = Node(current.x, current.y + 1);
        if (sq->southSquare != nullptr) neighbors[neighborCount++] = Node(current.x, current.y - 1);
        if (sq->eastSquare != nullptr)  neighbors[neighborCount++] = Node(current.x + 1, current.y);
        if (sq->westSquare != nullptr)  neighbors[neighborCount++] = Node(current.x - 1, current.y);
        //check walls and add counter for each direction
        // if (sq->northSquare != nullptr && sq->j + counter < 12) neighbors[neighborCount++] = Node(current.x, current.y + 1);
        // if (sq->southSquare != nullptr && sq->j - counter >= 0) neighbors[neighborCount++] = Node(current.x, current.y - 1);
        // if (sq->eastSquare != nullptr && sq->i + counter < 12)  neighbors[neighborCount++] = Node(current.x + 1, current.y);
        // if (sq->westSquare != nullptr && sq->i - counter >= 0)  neighbors[neighborCount++] = Node(current.x - 1, current.y);

        for (int i = 0; i < neighborCount; ++i) {
            Node n = neighbors[i];
            if (!visited[n.x][n.y]) {
                queue[back++] = n;
                visited[n.x][n.y] = true;
                parent[n.x][n.y] = current; // Stocke le parent
            }
        }
    }

    return Node(start.x(), start.y()); // Si aucune bombe trouvée, rester sur place
}


void move_to_next_bomb() {
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posIndex = coordinates_to_index(pos);

    Node nextMove = bfs_find_bomb(posIndex);
    gladiator->log(("nextMove: " + std::to_string(nextMove.x) + ", " + std::to_string(nextMove.y)).c_str());
    
    //si on est à la distance inférieur à erreurPos d'une bombe et qu'elle n'est pas dérriere un mur, on va dans sa direction
    Vector2 nextMovePos = index_to_coordinates(nextMove.x, nextMove.y);
    Vector2 nextMoveDir = nextMovePos - pos;
    if (nextMoveDir.norm2() < erreurPos) {
        gladiator->log("Going to bomb with first move at %f, %f", nextMovePos.x(), nextMovePos.y());
        go_to(nextMovePos, pos);
    }

    else if (nextMove.x != (int)posIndex.x() || nextMove.y != (int)posIndex.y()) {
        go_to(index_to_coordinates(nextMove.x, nextMove.y), pos);
    }
}


void bomb_placement()
{
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posIndex = coordinates_to_index(pos);
    int bomb_value = 0;
    int team = gladiator->robot->getData().teamId;
    int minimum_point_needed = 3;

    MazeSquare *mazeSquare = maze->getSquare(posIndex.x(), posIndex.y());
    if (mazeSquare != nullptr)
    {
        // On compte les cases valide
        if (mazeSquare != nullptr && mazeSquare->possession != team && !mazeSquare->danger)
            bomb_value++;
        if (mazeSquare->northSquare != nullptr && mazeSquare->northSquare->possession != team && !mazeSquare->northSquare->danger)
            bomb_value++;
        if (mazeSquare->southSquare != nullptr && mazeSquare->southSquare->possession != team && !mazeSquare->southSquare->danger)
            bomb_value++;
        if (mazeSquare->eastSquare != nullptr && mazeSquare->eastSquare->possession != team && !mazeSquare->eastSquare->danger)
            bomb_value++;
        if (mazeSquare->westSquare != nullptr && mazeSquare->westSquare->possession != team && !mazeSquare->westSquare->danger)
            bomb_value++;
    }
    if (gladiator->weapon->getBombCount() && bomb_value >= minimum_point_needed) {
        gladiator->weapon->dropBombs(1);
    }
}

void loop() {
    if (gladiator->game->isStarted()) {
        update_maze();
        move_to_next_bomb();
        bomb_placement();

    //     if (counter == -1){
    //         counter++;
    //         Serial.begin(9600); // Initialiser la communication série
    //     }

    //     unsigned long currentTime = millis(); // Temps actuel en millisecondes

    // // Vérifier si 14 secondes se sont écoulées
    //     if (currentTime - previousTime >= interval) {
    //         previousTime = currentTime; // Réinitialiser le timer
    //         counter++; // Incrémenter la variable
    //     }
    }
    delay(10);
}
