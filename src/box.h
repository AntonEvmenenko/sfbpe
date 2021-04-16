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
        m_q.segment(0, 2) = center;
        m_width = width;
        m_height = height;
        m_m = m;
        m_I = I;
    }

    void applyForce(Vector2d F, Vector2d F_p = Vector2d(0, 0)) {
        m_F = F;
        m_T_f = cross((Rotation2Dd(m_q(2)).toRotationMatrix() * F_p), F);
    }

    void applyTorque(double T) {m_T = T;}

    Vector2d getCenter() {return m_q.segment(0, 2);}
    Vector2d setCenter(Vector2d center) {m_q.segment(0, 2) = center;}

    double getAngle() {return m_q(2);}
    double setAngle(double angle) {m_q(2) = angle;}

    double getWidth() {return m_width;}
    double getHeight() {return m_height;}

    bool contains(Vector2d p) {
        Vector2d temp = Rotation2Dd(-m_q(2)).toRotationMatrix() * (p - m_q.segment(0, 2));
        return temp.x() >= -m_width / 2  && temp.x() <= m_width / 2 &&
               temp.y() >= -m_height / 2 && temp.y() <= m_height / 2;
    }

    Vector3d& q() {return m_q;}
    Vector3d& q_d() {return m_q_d;}
    Vector3d& q_dd() {return m_q_dd;}

    Vector3d M() {return Vector3d(m_m, m_m, m_I);}
    Vector3d F() {return Vector3d(m_F(0), m_F(1), m_T + m_T_f);}

private:
    // q = {x, y, angle}
    Vector3d m_q, m_q_d, m_q_dd;

    double m_width, m_height;
    double m_m;
    double m_I;

    Vector2d m_F;
    double m_T, m_T_f;
};

#endif // BOX_H