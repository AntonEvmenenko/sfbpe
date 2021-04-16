#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Dense>
#include "box.h"
#include "drawing.h"

using Eigen::MatrixXd;

class Constraint {
public:
    Constraint() {}

    Constraint(Box *a, Box *b, Vector2d anchoringPointA, Vector2d anchoringPointB) {
        this->a = a;
        this->b = b;
        this->anchoringPointA = anchoringPointA;
        this->anchoringPointB = anchoringPointB;
    }

    MatrixXd getJ() {
        MatrixXd J(2, 6);

        Vector2d r1 = Rotation2Dd(a->getAngle()).toRotationMatrix() * (anchoringPointA);
        Vector2d r2 = Rotation2Dd(b->getAngle()).toRotationMatrix() * (anchoringPointB);

        drawPoint(a->getCenter() + r1);
        drawPoint(b->getCenter() + r2);

        Vector3d q_d_a = a->q_d();
        Vector3d q_d_b = b->q_d();

        J << 1, 0, -r1(1), -1,  0,  r2(1),
             0, 1,  r1(0),  0, -1, -r2(0);

        return J;
    }

    MatrixXd getJd() {
        MatrixXd J_d(2, 6);

        Vector2d r1 = Rotation2Dd(a->getAngle()).toRotationMatrix() * (anchoringPointA);
        Vector2d r2 = Rotation2Dd(b->getAngle()).toRotationMatrix() * (anchoringPointB);

        Vector3d q_d_a = a->q_d();
        Vector3d q_d_b = b->q_d();

        J_d << 0, 0, -q_d_a(2) * r1(0), 0, 0, q_d_b(2) * r2(0),
               0, 0, -q_d_a(2) * r1(1), 0, 0, q_d_b(2) * r2(1);

        return J_d;
    }

    Box *a;
    Box *b;

private:
    Vector2d anchoringPointA;
    Vector2d anchoringPointB;
};

#endif // CONSTRAINT_H