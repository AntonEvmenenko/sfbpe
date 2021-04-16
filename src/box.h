#ifndef BOX_H
#define BOX_H

#include <Eigen/Dense>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Rotation2Dd;

double cross(Vector2d a, Vector2d b);

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

    Vector3d getM() {return Vector3d(m, m, I);}

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

    Vector3d& a() {return q_d;}

    Vector3d q_d, q_dd;

private:
    Vector2d center;
    double width, height;
    double angle;
    double m;
    double I;

    Vector2d F;
    double T, T_f;
};

#endif // BOX_H