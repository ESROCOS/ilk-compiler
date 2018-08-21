#include <iostream>

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "ur5.h"

using namespace std;
using namespace ur5;

static kul::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

int main(int argc, char** argv)
{
    if(argc < 2) {
        std::cerr << "Please provide a dataset file." << std::endl;
        return -1;
    }

    kul::TextDataset data(argv[1]);

    ur5::joint_state q;
    kul::AxisAngle Rdiff;
    double err_pos = 0;
    double err_ori = 0;
    int count = 0;
    while( ! data.eof() )
    {
        data.readVector(ur5::dofs_count, q);
        kul::pose_t given_1, computed_1;
        data.readPose(given_1);
        kul::pose_t given_2, computed_2;
        data.readPose(given_2);
        kul::pose_t given_3, computed_3;
        data.readPose(given_3);
        kul::pose_t given_4, computed_4;
        data.readPose(given_4);
        fk1(kk, q,
                       computed_1,
                       computed_2,
                       computed_3,
                       computed_4);
        Rdiff = kul::orientationDistance(
            kul::eg_get_rotation(computed_1),
            kul::eg_get_rotation(given_1));
        err_ori += Rdiff.angle;
        err_pos += (kul::eg_get_position(computed_1-given_1)).norm();
        count++;
        Rdiff = kul::orientationDistance(
            kul::eg_get_rotation(computed_2),
            kul::eg_get_rotation(given_2));
        err_ori += Rdiff.angle;
        err_pos += (kul::eg_get_position(computed_2-given_2)).norm();
        count++;
        Rdiff = kul::orientationDistance(
            kul::eg_get_rotation(computed_3),
            kul::eg_get_rotation(given_3));
        err_ori += Rdiff.angle;
        err_pos += (kul::eg_get_position(computed_3-given_3)).norm();
        count++;
        Rdiff = kul::orientationDistance(
            kul::eg_get_rotation(computed_4),
            kul::eg_get_rotation(given_4));
        err_ori += Rdiff.angle;
        err_pos += (kul::eg_get_position(computed_4-given_4)).norm();
        count++;
    }

    err_pos /= count;
    err_ori /= count;

    std::cout << "Average position error   : " << err_pos << std::endl;
    std::cout << "Average orientation error: " << err_ori << std::endl;

    return 0;
}
