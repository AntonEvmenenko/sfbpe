#include "pgs.h"

VectorXd pgs(Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &lo, Eigen::VectorXd &hi, int kMax)
{
    //A is a m-n matrix, m rows, n columns
    if (A.rows() != b.rows())
        return b;

    int i, j, n = A.rows();

    Eigen::VectorXd x(b.rows());
    x.setZero();

    double delta;

    // Gauss-Seidel Solver
    for (int k = 0; k < kMax; k++)
    {
        for (i = 0; i < n; i++)
        {
            delta = 0.0;

            for (j = 0; j < i; j++)
                delta += A(i, j) * x[j];
            for (j = i + 1; j < n; j++)
                delta += A(i, j) * x[j];

            double aDiag = A(i, i);
            x[i] = (b[i] - delta) / A(i, i);
            if (x[i] < lo[i])
                x[i] = lo[i];
            if (x[i] > hi[i])
                x[i] = hi[i];
        }
    }

    return x;
}