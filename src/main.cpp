#include "gladiator.h"
#include <cmath>
#undef abs

Gladiator *gladiator;
Maze *maze;

int MAZE_SIZE = 3;
int CELL_SIZE = 3 /

float kw = 0.1; //kw et kv sont des constantes de réglage du correcteur qu'il faut ajuster avec des tests
float kv = 0.1; // Plus elles sont grandes, plus le robot va vite mais plus il risque de ne pas s'arrêter à temps
float wlimit = 3.f; // wlimit et vlimit sont les limites de vitesse angulaire et linéaire du robot
float vlimit = 0.6;
float erreurPos = 0.07; // erreurPos est la distance minimale à laquelle le robot doit être de la consigne pour s'arrêter

//store the maze square where i already went 
int MAZE_TRACK[12][12] = {0};

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

    if (d > erreurPos)
    {
        double rho = atan2(delta.y(), delta.x());
        double consw = kw * reductionAngle(rho - gladiator->robot->getData().position.a);

        double consv = kv * d * cos(reductionAngle(rho - gladiator->robot->getData().position.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    maze = new Maze();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

//Index to coordinates
Vector2 index_to_coordinates(int i, int j)
{
    return Vector2(i * CELL_SIZE + CELL_SIZE / 2, j * CELL_SIZE + CELL_SIZE / 2);
}

//Coordinates to index
Vector2 coordinates_to_index(Vector2 pos)
{
    return Vector2((int)(pos.x() / CELL_SIZE), (int)(pos.y() / CELL_SIZE));
}

//If we are close to the center of a maze square, we are considered to be in the square and we can update the maze array
void update_maze()
{
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posIndex = coordinates_to_index(pos);
    Vector2 target = index_to_coordinates(posIndex.x(), posIndex.y());
    gladiator->log(("Position : " + std::to_string(pos.x()) + " " + std::to_string(pos.y())).c_str());

    // Vérifier si on est proche du centre de la case
    if ((pos - target).norm2() < erreurPos)
    {
        MAZE_TRACK[(int)posIndex.x()][(int)posIndex.y()] = 1;
    }
}


//Search if there is a maze square available next to us thatwe never went to. If not go in the direction of the center of the maze until we find one
//We can have sqaure maze info with maze.getSquare(i, j) and  it return a MazeSquare object
//If we have a MazeSquare object we can get the walls with mazeSquare.north(), mazeSquare.south() and so on ans return a null pointer if there is a wall else return a pointer to the next square
//We can also get the position of the square with mazeSquare.i and mazeSquare.
void search_next_square()
{
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posIndex = coordinates_to_index(pos);

    MazeSquare *mazeSquare = maze->getSquare(posIndex.x(), posIndex.y());
    if (mazeSquare != nullptr)
    {
        // On cherche une case accessible et non encore visitée
        if (mazeSquare->northSquare != nullptr && MAZE_TRACK[(int)posIndex.x()][(int)posIndex.y() - 1] == 0)
        {
            go_to(index_to_coordinates((int)posIndex.x(), (int)posIndex.y() - 1), pos);
        }
        else if (mazeSquare->southSquare != nullptr && MAZE_TRACK[(int)posIndex.x()][(int)posIndex.y() + 1] == 0)
        {
            go_to(index_to_coordinates((int)posIndex.x(), (int)posIndex.y() + 1), pos);
        }
        else if (mazeSquare->eastSquare != nullptr && MAZE_TRACK[(int)posIndex.x() + 1][(int)posIndex.y()] == 0)
        {
            go_to(index_to_coordinates((int)posIndex.x() + 1, (int)posIndex.y()), pos);
        }
        else if (mazeSquare->westSquare != nullptr && MAZE_TRACK[(int)posIndex.x() - 1][(int)posIndex.y()] == 0)
        {
            go_to(index_to_coordinates((int)posIndex.x() - 1, (int)posIndex.y()), pos);
        }
        else
        {
            go_to(index_to_coordinates(6, 6), pos);
        }
    }
    else
    {
        go_to(index_to_coordinates(6, 6), pos);
    }
}


void loop()
{
    if (gladiator->game->isStarted())
    {
        update_maze();  // Mise à jour du suivi du labyrinthe
        search_next_square();  // Recherche d'une nouvelle case
    }
    delay(500);
}
