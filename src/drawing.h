#ifndef DRAWING_H
#define DRAWING_H

#include <SDL2/SDL.h>
#include <Eigen/Dense>

#include "box.h"

using Eigen::Vector2d;
using Eigen::Rotation2Dd;

#define SCREEN_WIDTH 900
#define SCREEN_HEIGHT 900

extern SDL_Renderer *ren;

Vector2d cs(Vector2d v);
Vector2d csi(Vector2d v);

void drawLine(Vector2d a, Vector2d b);
void draw(Box &box, bool selected = false);
void drawArrow(Vector2d a, Vector2d b);

#endif // DRAWING_H