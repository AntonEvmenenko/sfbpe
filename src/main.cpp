#include <SDL2/SDL.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <iomanip>

#include <Eigen/Dense>

#include "box.h"
#include "drawing.h"
#include "solver.h"
#include "constraint.h"

using namespace std;

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Rotation2Dd;

SDL_Window *win = NULL;
SDL_Renderer *ren = NULL;

bool init() {
    bool ok = true;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cout << "Can't init SDL: " << SDL_GetError() << endl;
    }

    win = SDL_CreateWindow("sfbpe", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (win == NULL) {
        cout << "Can't create window: " << SDL_GetError() << endl;
        ok = false;
    }

    ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (ren == NULL) {
        cout << "Can't create renderer: " << SDL_GetError() << endl;
        ok = false;
    }
    return ok;
}

void quit() {
    SDL_DestroyWindow(win);
    win = NULL;

    SDL_DestroyRenderer(ren);
    ren = NULL;

    SDL_Quit;
}

VectorXd F_c;

vector<Box> bodies;
vector<Constraint> constraints;

int main (int arhc, char ** argv) {
    if (!init()) {
        quit();
        system("pause");
        return 1;
    }

    bodies.push_back(Box(Vector2d(-0.05, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.05, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.15, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.25, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.35, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.45, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.55, 0), 0.15, 0.05, 1, 0.01));
    bodies.push_back(Box(Vector2d(0.65, 0), 0.15, 0.05, 1, 0.01));

    constraints.push_back(Constraint(addressof(bodies[0]), addressof(bodies[1]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[1]), addressof(bodies[2]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[2]), addressof(bodies[3]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[3]), addressof(bodies[4]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[4]), addressof(bodies[5]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[5]), addressof(bodies[6]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));
    constraints.push_back(Constraint(addressof(bodies[6]), addressof(bodies[7]), Vector2d(0.05, 0), Vector2d(-0.05, 0)));

    F_c = VectorXd(bodies.size() * 3);
    F_c.setZero();

    Vector2d F_m;
    Box *selectedBox = nullptr;
    Vector2d pullPointLocal;

    bool run = true;
    bool leftMouseButtonPressed = false;
    SDL_Event e;

    while (run) {
        while(SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                run = false;
            }
        }

        int x, y;

        if (SDL_GetMouseState(&x, &y) & SDL_BUTTON(SDL_BUTTON_LEFT)) {
            if (!leftMouseButtonPressed) {
                Vector2d pullPoint = csi(Vector2d(x, y));

                for (auto body = bodies.begin(); body != bodies.end(); body++) {
                    if (body->contains(pullPoint)) {
                        selectedBox = addressof(*body);
                    }
                }

                if (selectedBox) {
                    pullPointLocal = Rotation2Dd(-selectedBox->getAngle()).toRotationMatrix() * (pullPoint - selectedBox->getCenter());
                }

                leftMouseButtonPressed = true;
            } else {
                Vector2d pullPointGlobal = selectedBox->getCenter() + Rotation2Dd(selectedBox->getAngle()).toRotationMatrix() * pullPointLocal;
                F_m = csi(Vector2d(x, y)) - pullPointGlobal;
                selectedBox->applyForce(F_m, pullPointLocal);
            }
        } else {
            F_m = Vector2d(0, 0);
            if (selectedBox) {
                selectedBox->applyForce(F_m);
            }
            
            leftMouseButtonPressed = false;
            selectedBox = nullptr;
        }

        SDL_SetRenderDrawColor(ren, 0xFF, 0xFF, 0xFF, 0xFF);
        SDL_RenderClear(ren);
        SDL_SetRenderDrawColor(ren, 0x00, 0x00, 0x00, 0x00);

        for (auto body = bodies.begin(); body != bodies.end(); body++) {
            draw(*body, selectedBox == addressof(*body));
        }

        if (selectedBox) {
            Vector2d pullPointGlobal = selectedBox->getCenter() + Rotation2Dd(selectedBox->getAngle()).toRotationMatrix() * pullPointLocal;
            drawArrow(pullPointGlobal, pullPointGlobal + F_m);
        }

        solve();
        update(0.01);

        SDL_RenderPresent(ren);
        SDL_Delay(10);
    }

    quit();
    return 0;
}