#ifndef PGS_H
#define PGS_H

#include <Eigen/Dense>

#include "box.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd pgs(Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &lo, Eigen::VectorXd &hi, int kMax = 5);

#endif // PGS_H