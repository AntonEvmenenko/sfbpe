#include "box.h"

double cross(Vector2d a, Vector2d b) {
    return a.x() * b.y() - a.y() * b.x();
}