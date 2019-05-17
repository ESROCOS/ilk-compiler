#include <iostream>

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "ur5.h"

static kul::RoundTheDifference<double> rounder(0.1);
static ur5::ModelConstants mc;

int main(int argc, char** argv)
{
    if(argc < 2) {
        std::cerr << "Please provide a dataset file." << std::endl;
        return -1;
    }
    kul::NaiveBinDataset data(argv[1]);

    kul::AxisAngle Rdiff;
    double err_pos = 0;
    double err_ori = 0;
    int count = 0;

    ur5::joint_state q;
    kul::pose_t fr_wrist_3__fr_base;

    kul::pose_t given_1;

    while( ! data.eof() )
    {
        data.readVector(ur5::dofs_count, q);
        data.readPose(given_1);
        ur5::fk1(mc, q, fr_wrist_3__fr_base);

        Rdiff = kul::orientationDistance(
                    kul::eg_get_rotation(fr_wrist_3__fr_base),
                    kul::eg_get_rotation(given_1));
        err_ori += Rdiff.angle;
        err_pos += (kul::eg_get_position(fr_wrist_3__fr_base-given_1)).norm();
        count++;
    }

    err_pos /= count;
    err_ori /= count;

    std::cout << "Average position error   : " << err_pos << std::endl;
    std::cout << "Average orientation error: " << err_ori << std::endl;

    return 0;
}
