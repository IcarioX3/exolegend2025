// #include "gladiator.h"
// #include <cmath>
// #undef abs

// Gladiator *gladiator;
// Maze *maze;

// int MAZE_SIZE = 3;
// float CELL_SIZE = MAZE_SIZE / 12.f;

// //kw influe sur la vitesse de rotation du robot, kv influe sur la vitesse de translation du robot
// float kw = 1.5; //kw et kv sont des constantes de réglage du correcteur qu'il faut ajuster avec des tests
// float kv = 0.7; // Plus elles sont grandes, plus le robot va vite mais plus il risque de ne pas s'arrêter à temps
// float wlimit = 3.f; // wlimit et vlimit sont les limites de vitesse angulaire et linéaire du robot
// float vlimit = 0.6;
// float erreurPos = 0.13; // erreurPos est la distance minimale à laquelle le robot doit être de la consigne pour s'arrêter


// class Vector2
// {
//   public:
//     Vector2() : _x(0.), _y(0.)
//     {
//     }
//     Vector2(float x, float y) : _x(x), _y(y)
//     {
//     }

//     float norm1() const
//     {
//         return std::abs(_x) + std::abs(_y);
//     }
//     float norm2() const
//     {
//         return std::sqrt(_x * _x + _y * _y);
//     }
//     void normalize()
//     {
//         _x /= norm2();
//         _y /= norm2();
//     }
//     Vector2 normalized() const
//     {
//         Vector2 out = *this;
//         out.normalize();
//         return out;
//     }

//     Vector2 operator-(const Vector2 &other) const
//     {
//         return {_x - other._x, _y - other._y};
//     }
//     Vector2 operator+(const Vector2 &other) const
//     {
//         return {_x + other._x, _y + other._y};
//     }
//     Vector2 operator*(float f) const
//     {
//         return {_x * f, _y * f};
//     }

//     bool operator==(const Vector2 &other) const
//     {
//         return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5;
//     }
//     bool operator!=(const Vector2 &other) const
//     {
//         return !(*this == other);
//     }

//     float x() const
//     {
//         return _x;
//     }
//     float y() const
//     {
//         return _y;
//     }

//     float dot(const Vector2 &other) const
//     {
//         return _x * other._x + _y * other._y;
//     }
//     float cross(const Vector2 &other) const
//     {
//         return _x * other._y - _y * other._x;
//     }
//     float angle(const Vector2 &m) const
//     {
//         return std::atan2(cross(m), dot(m));
//     }
//     float angle() const
//     {
//         return std::atan2(_y, _x);
//     }

//   private:
//     float _x, _y;
// };

// double reductionAngle(double x)
// {
//     x = fmod(x + PI, 2 * PI);
//     if (x < 0)
//         x += 2 * PI;
//     return x - PI;
// }
// void go_to(Vector2 cons, Vector2 pos)
// {
//     double consvl, consvr;
//     Vector2 delta = cons - pos;
//     double d = delta.norm2();

//     double rho = atan2(delta.y(), delta.x());
//     double consw = kw * reductionAngle(rho - gladiator->robot->getData().position.a);

//     double consv = kv * d * cos(reductionAngle(rho - gladiator->robot->getData().position.a));
//     consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
//     consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

//     consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
//     consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2

//     gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
//     gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
// }

// void reset()
// {
//     // fonction de reset:
//     // initialisation de toutes vos variables avant le début d'un match
//     gladiator->log("Call of reset function"); // GFA 4.5.1

// }

// void setup()
// {
//     // instanciation de l'objet gladiator
//     gladiator = new Gladiator();
//     maze = new Maze();
//     // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
//     gladiator->game->onReset(&reset); // GFA 4.4.1
// }

// //Index to coordinates
// Vector2 index_to_coordinates(int i, int j)
// {
//     return Vector2(i * CELL_SIZE + CELL_SIZE / 2, j * CELL_SIZE + CELL_SIZE / 2);
// }

// //Coordinates to index
// Vector2 coordinates_to_index(Vector2 pos)
// {
//     return Vector2((int)(pos.x() / CELL_SIZE), (int)(pos.y() / CELL_SIZE));
// }

// Vector2 getClosestBomb()
// {
//     Vector2 pos = Vector2(gladiator->robot->getData().position.x, gladiator->robot->getData().position.y);
//     Vector2 closestBomb = Vector2(1.5, 1.5); //Initialisation à la valeur du centre du labyrinthe

//     for (int i = 0; i < MAZE_SIZE; i++)
//     {
//         for (int j = 0; j < MAZE_SIZE; j++)
//         {
//             MazeSquare *mazeSquare = maze->getSquare(i, j);
//             if (mazeSquare != nullptr)
//             {
//                 if (mazeSquare->coin.value > 0)
//                 {
//                     Vector2 bomb = index_to_coordinates(i, j);
//                     if ((bomb - pos).norm2() < (closestBomb - pos).norm2())
//                     {
//                         closestBomb = bomb;
//                     }
//                 }
//             }
//         }
//     }
//     return closestBomb;
// }

// void loop()
// {
//     if (gladiator->game->isStarted())
//     {
//         Vector2 pos = Vector2(gladiator->robot->getData().position.x, gladiator->robot->getData().position.y);

//         Vector2 target = getClosestBomb();
//         if (target == Vector2(1.5, 1.5))
//         {
//             gladiator->log("No bomb found");
//             return;
//         }

//         // Vérifier si on est proche du centre de la case
//         if ((pos - target).norm2() < erreurPos)
//         {
//             gladiator->log("Bomb found");
//             return;
//         }

//         go_to(target, pos);
//     }
//     delay(100);
// }
