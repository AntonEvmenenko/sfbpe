#include <SDL2/SDL.h>
#include <iostream>
#include <math.h>

#include <Eigen/Dense>

using namespace std;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Rotation2Dd;
using Eigen::MatrixXd;
using Eigen::DiagonalMatrix;

int SCREEN_WIDTH = 900;
int SCREEN_HEIGHT = 900;

SDL_Window *win = NULL;
SDL_Renderer *ren = NULL;

bool init() {
    bool ok = true;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cout << "Can't init SDL: " << SDL_GetError() << endl;
    }

    win = SDL_CreateWindow("Примитивы", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
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

Vector2d cs(Vector2d v) {
    return Vector2d(SCREEN_WIDTH / 2.0 + v(0) * SCREEN_WIDTH / 2.0, 
                    SCREEN_HEIGHT / 2.0 - v(1) * SCREEN_HEIGHT / 2.0);
}

Vector2d csi(Vector2d v) {
    return Vector2d(v(0) / SCREEN_WIDTH * 2.0 - 1.0, -(v(1) / SCREEN_HEIGHT * 2.0 - 1.0));
}

double cross(Vector2d a, Vector2d b) {
    return a.x() * b.y() - a.y() * b.x();
}

class Box {
public:
    Box() {}

    Box(Vector2d center, double width, double height, double m, double I) {
        this->center = center;
        this->width = width;
        this->height = height;
        this->m = m;
        this->I = I;
    }

    void applyForce(Vector2d F, Vector2d F_p = Vector2d(0, 0)) {
        this->F = F;
        T_f = cross((Rotation2Dd(angle).toRotationMatrix() * F_p), F);
    }

    void applyTorque(double T) {this->T = T;}

    Vector2d getCenter() {return center;}
    Vector2d setCenter(Vector2d center) {this->center = center;}

    double getWidth() {return width;}
    double getHeight() {return height;}

    double getAngle() {return angle;}
    double setAngle(double angle) {this->angle = angle;}

    double getM() {return m;}
    double getI() {return I;}

    Vector3d getF() {return Vector3d(F(0), F(1), T + T_f);}

    bool contains(Vector2d p) {
        Vector2d temp = Rotation2Dd(-angle).toRotationMatrix() * (p - center);
        return temp.x() >= -width / 2  && temp.x() <= width / 2 &&
               temp.y() >= -height / 2 && temp.y() <= height / 2;
    }

    Vector3d getQ() {
        return Vector3d(center.x(), center.y(), angle);
    }
    
    void setQ(Vector3d q) {
        center = Vector2d(q(0), q(1));
        angle = q(2);
    }

    void setQd(Vector3d q_d) {this->q_d = q_d;}
    Vector3d getQd() {return q_d;}

    void setQdd(Vector3d q_dd) {this->q_dd = q_dd;}
    Vector3d getQdd() {return q_dd;}

private:
    Vector2d center;
    double width, height;
    double angle;
    double m;
    double I;

    Vector2d F;
    double T, T_f;

    Vector3d q_d, q_dd;
};

class Constraint {
public:
    Box *a;
    Box *b;

    Vector2d anchoringPointA;
    Vector2d anchoringPointB;
};

void drawLine(Vector2d a, Vector2d b) {

    SDL_RenderDrawLine(ren, cs(a)(0), cs(a)(1), cs(b)(0), cs(b)(1));
}

void draw(Box &box, bool selected = false) {
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

Box a, b, *selectedBox = nullptr;
Constraint c;

VectorXd q(6), q_d(6), q_dd(6);
DiagonalMatrix<double, 6> M;
VectorXd F_ext(6), F_c(6);

Vector2d F_m;
Vector2d pullPointLocal;

void update(double dt) {
    q << a.getQ(), b.getQ();

    F_ext << a.getF(), b.getF();

    M.diagonal() << a.getM(), a.getM(), a.getI(), b.getM(), b.getM(), b.getI();

    VectorXd q_dot(6);
    q_dd = M.inverse() * (F_ext + F_c);

    q_d += q_dd * dt;
    q += q_d * dt;

    a.setQ(Vector3d(q(0), q(1), q(2)));
    b.setQ(Vector3d(q(3), q(4), q(5)));
}

void solve() {
    Vector2d r1 = Rotation2Dd(c.a->getAngle()).toRotationMatrix() * (c.anchoringPointA);
    Vector2d r2 = Rotation2Dd(c.b->getAngle()).toRotationMatrix() * (c.anchoringPointB);

    a.setCenter(Vector2d(q(0), q(1)));
    a.setAngle(q(2));
    b.setCenter(Vector2d(q(3), q(4)));
    b.setAngle(q(5));

    drawArrow(a.getCenter(), a.getCenter() + r1);
    drawArrow(b.getCenter(), b.getCenter() + r2);

    MatrixXd J(2, 6), J_d(2, 6);
    J << 1, 0, -r1(1), -1,  0,  r2(1),
         0, 1,  r1(0),  0, -1, -r2(0);

    J_d << 0, 0, -q_d(2) * r1(0), 0, 0, q_d(5) * r2(0),
           0, 0, -q_d(2) * r1(1), 0, 0, q_d(5) * r2(1);

    MatrixXd A;
    VectorXd b, lambda;
    A = J * M.inverse() * J.transpose();
    b = -J_d * q_d - J * M.inverse() * F_ext - (1 / 0.01 * J * q_d);

    lambda = A.colPivHouseholderQr().solve(b);

    F_c = J.transpose() * lambda;
}

int main (int arhc, char ** argv) {
    if (!init()) {
        quit();
        system("pause");
        return 1;
    }

    a = Box(Vector2d(0.1, 0), 0.3, 0.1, 1, 0.1);
    b = Box(Vector2d(-0.1, 0), 0.3, 0.1, 1, 0.1);

    c.a = &a;
    c.b = &b;
    c.anchoringPointA = Vector2d(-0.1, 0);
    c.anchoringPointB = Vector2d(0.1, 0);

    update(0.1);

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
                // a.F_p = pullPointLocal;
                if (a.contains(pullPoint)) {
                    leftMouseButtonPressed = true;
                    selectedBox = &a;
                } else if (b.contains(pullPoint)) {
                    leftMouseButtonPressed = true;
                    selectedBox = &b;
                }
                if (selectedBox) {
                    pullPointLocal = Rotation2Dd(-selectedBox->getAngle()).toRotationMatrix() * (pullPoint - selectedBox->getCenter());
                }
            } else {
                if (leftMouseButtonPressed) {
                    Vector2d pullPointGlobal = selectedBox->getCenter() + Rotation2Dd(selectedBox->getAngle()).toRotationMatrix() * pullPointLocal;
                    F_m = csi(Vector2d(x, y)) - pullPointGlobal;
                    // a.F = F_m * 1;
                    selectedBox->applyForce(F_m, pullPointLocal);
                }
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

        draw(a, selectedBox == &a);
        draw(b, selectedBox == &b);

        if (selectedBox) {
            Vector2d pullPointGlobal = selectedBox->getCenter() + Rotation2Dd(selectedBox->getAngle()).toRotationMatrix() * pullPointLocal;
            drawArrow(pullPointGlobal, pullPointGlobal + F_m);
        }

        solve();
        update(0.01);

        // double E = (M * (q_d.cwiseProduct(q_d)) / 2).sum();
        // std::cout << E << std::endl;

        SDL_RenderPresent(ren);
        SDL_Delay(10);
    }

    quit();
    return 0;
}