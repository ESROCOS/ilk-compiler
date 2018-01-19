
/*
Rhz(a) := matrix(
             [cos(a),-sin(a),0,0],
             [sin(a), cos(a),0,0],
             [0,      0,     1,0],
             [0,      0,     0,1]);

Th(t) := matrix([1,0,0,t[1]], [0,1,0,t[2]], [0,0,1,t[3]],[0,0,0,1]);
*/

#include "joint-transforms.h"

void rot_z__a_x_b(double arg, hom_t_t& out)
{
    out.setIdentity();
    double s = std::sin(arg);
    double c = std::cos(arg);

    out(0,0) = c;
    out(0,1) = -s;
    out(1,0) = s;
    out(1,1) = c;
}

void rot_z__b_x_a(double arg, hom_t_t& out)
{
    out.setIdentity();
    double s = std::sin(arg);
    double c = std::cos(arg);

    out(0,0) = c;
    out(0,1) = s;
    out(1,0) = -s;
    out(1,1) = c;
}

void tr_z__a_x_b(double arg, hom_t_t& out)
{
    out.setIdentity();
    out(2,3) = arg;
}

void tr_z__b_x_a(double arg, hom_t_t& out)
{
    out.setIdentity();
    out(2,3) = -arg;
}
