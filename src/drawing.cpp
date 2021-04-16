#include "drawing.h"

Vector2d cs(Vector2d v) {
    return Vector2d(SCREEN_WIDTH / 2.0 + v(0) * SCREEN_WIDTH / 2.0, 
                    SCREEN_HEIGHT / 2.0 - v(1) * SCREEN_HEIGHT / 2.0);
}

Vector2d csi(Vector2d v) {
    return Vector2d(v(0) / SCREEN_WIDTH * 2.0 - 1.0, -(v(1) / SCREEN_HEIGHT * 2.0 - 1.0));
}

void drawLine(Vector2d a, Vector2d b) {

    SDL_RenderDrawLine(ren, cs(a)(0), cs(a)(1), cs(b)(0), cs(b)(1));
}

void draw(Box &box, bool selected) {
    SDL_Color currentColor;
    SDL_GetRenderDrawColor(ren, &(currentColor.r), &(currentColor.g), &(currentColor.b), &(currentColor.a));
    if (selected) {
        SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0xFF, 0xFF);
    } else {
        SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0x00, 0xFF);
    }

    Rotation2Dd::Matrix2 R = Rotation2Dd(box.getAngle()).toRotationMatrix();

    Vector2d topLeftCorner = box.getCenter() + R * Vector2d(-box.getWidth() / 2, box.getHeight() / 2);
    Vector2d topRightCorner = box.getCenter() + R * Vector2d(box.getWidth() / 2, box.getHeight() / 2);
    Vector2d bottomRightCorner = box.getCenter() + R * Vector2d(box.getWidth() / 2, -box.getHeight() / 2);
    Vector2d bottomLeftCorner = box.getCenter() + R * Vector2d(-box.getWidth() / 2, -box.getHeight() / 2);

    drawLine(topLeftCorner, topRightCorner);
    drawLine(topRightCorner, bottomRightCorner);
    drawLine(bottomRightCorner, bottomLeftCorner);
    drawLine(bottomLeftCorner, topLeftCorner);

    SDL_SetRenderDrawColor(ren, currentColor.r, currentColor.g, currentColor.b, currentColor.a);
}

void drawArrow(Vector2d a, Vector2d b) {
    const double angle = M_PI / 8.0;
    const double length = 0.02;

    SDL_Color currentColor;
    SDL_GetRenderDrawColor(ren, &(currentColor.r), &(currentColor.g), &(currentColor.b), &(currentColor.a));
    SDL_SetRenderDrawColor(ren, 0xFF, 0x00, 0x00, 0xFF);

    Vector2d ba = a - b;
    Vector2d p1 = (Rotation2Dd(angle).toRotationMatrix() * ba).normalized() * length;
    Vector2d p2 = (Rotation2Dd(-angle).toRotationMatrix() * ba).normalized() * length;

    drawLine(a, b);
    drawLine(b, b + p1);
    drawLine(b, b + p2);

    SDL_SetRenderDrawColor(ren, currentColor.r, currentColor.g, currentColor.b, currentColor.a);
}