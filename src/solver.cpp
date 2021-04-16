#include "solver.h"
#include <Eigen/Dense>

#include "vector"
#include "box.h"
#include "constraint.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

extern vector<Box> bodies;
extern VectorXd F_c;
extern vector<Constraint> constraints;

void update(double dt) {
    VectorXd q(bodies.size() * 3);
    VectorXd q_d(bodies.size() * 3);
    VectorXd q_dd(bodies.size() * 3);

    VectorXd M(bodies.size() * 3);
    VectorXd F(bodies.size() * 3);

    for (auto body = bodies.begin(); body != bodies.end(); body++) {
        q.segment((body - bodies.begin()) * 3, 3) = body->getQ();
        q_d.segment((body - bodies.begin()) * 3, 3) = body->q_d;
        q_dd.segment((body - bodies.begin()) * 3, 3) = body->q_dd;

        M.segment((body - bodies.begin()) * 3, 3) = body->getM();
        F.segment((body - bodies.begin()) * 3, 3) = body->getF();
    }

    q_dd = M.asDiagonal().inverse() * (F + F_c);
    q_d += q_dd * dt;
    q += q_d * dt;

    for (auto body = bodies.begin(); body != bodies.end(); body++) {
        body->setQ(q.segment((body - bodies.begin()) * 3, 3));
        body->q_d = q_d.segment((body - bodies.begin()) * 3, 3);
        body->q_dd = q_dd.segment((body - bodies.begin()) * 3, 3);
    }
}

void solve() {
    VectorXd M(bodies.size() * 3);
    VectorXd q_d(bodies.size() * 3);
    VectorXd F_ext(bodies.size() * 3);
    
    for (auto body = bodies.begin(); body != bodies.end(); body++) {
        q_d.segment((body - bodies.begin()) * 3, 3) = body->q_d;
        M.segment((body - bodies.begin()) * 3, 3) = body->getM();
        F_ext.segment((body - bodies.begin()) * 3, 3) = body->getF();
    }

    MatrixXd J(constraints.size() * 2, bodies.size() * 3), J_d(constraints.size() * 2, bodies.size() * 3);

    J.setZero();
    J_d.setZero();

    for (auto c = constraints.begin(); c != constraints.end(); c++) {
        MatrixXd J_current = c->getJ();
        MatrixXd J_d_current = c->getJd();

        for (auto b = bodies.begin(); b != bodies.end(); b++) {
            if (addressof(*b) == c->a) {
                J.block((c - constraints.begin()) * 2, (b - bodies.begin()) * 3, 2, 3) = J_current.block(0, 0, 2, 3);
                J_d.block((c - constraints.begin()) * 2, (b - bodies.begin()) * 3, 2, 3) = J_d_current.block(0, 0, 2, 3);
            } else if (addressof(*b) == c->b) {
                J.block((c - constraints.begin()) * 2, (b - bodies.begin()) * 3, 2, 3) = J_current.block(0, 3, 2, 3);
                J_d.block((c - constraints.begin()) * 2, (b - bodies.begin()) * 3, 2, 3) = J_d_current.block(0, 3, 2, 3);
            }
        }
    }

    MatrixXd A;
    VectorXd b, lambda;

    A = J * M.asDiagonal().inverse() * J.transpose();
    b = -J_d * q_d - J * M.asDiagonal().inverse() * F_ext - (0.1 / 0.01 * J * q_d);

    lambda = A.colPivHouseholderQr().solve(b);

    F_c = J.transpose() * lambda;
}