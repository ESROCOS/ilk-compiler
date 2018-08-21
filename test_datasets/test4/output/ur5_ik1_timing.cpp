#include "ur5.h"

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <iostream>
#include <fstream>

#include <time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;
    
using namespace std;
using namespace ur5;
using namespace kul;


static kul::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

int main(int argc, char** argv)
{
    int different_dataset_repetitions = 10;
    int solver_call_repetitions_per_dataset = 10000000;
    if (argc >= 2) {
        different_dataset_repetitions = atoi(argv[1]);
    }
    if (argc >= 3) {
        solver_call_repetitions_per_dataset = atoi(argv[2]);
    }
    if (different_dataset_repetitions <= 0 || solver_call_repetitions_per_dataset <= 0) {
        std::cerr << "Invalid parameters." << std::endl;
        std::cerr << "Proper parameters:  [number of random datasets to generate] [number of solver calls per dataset]" << std::endl;
        return -1;
    }

    cout << "Execution times of blocks of " << solver_call_repetitions_per_dataset << " solver calls will be measured " << different_dataset_repetitions << " times for different datasets. Please wait..." << endl;


    

    ur5::joint_state q;
    kul::AxisAngle Rdiff;
    int count = 0;

    accumulator_set<double, stats<tag::variance> > acc;

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
        joint_state q;
        kul::pose_t fr_wrist_3__wrt__fr_base;
        Jacobian_t J;
        joint_state qd, qd_ik;
        qd.setRandom();
        q.setRandom();

        struct timespec t_before;
        struct timespec t_after;
        double t_delta;
    
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_before);

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
             fk_for_ik1(kk, q, fr_wrist_3__wrt__fr_base, J);
             vector6_t twist = J * qd;
             ik1(kk, q, kul::linearCoords(twist), qd_ik);
        }
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_after);

        double before_in_ns = t_before.tv_nsec + t_before.tv_sec * 1e9;
        double after_in_ns = t_after.tv_nsec + t_after.tv_sec * 1e9;
        t_delta = after_in_ns - before_in_ns;
        acc(t_delta);
    

        count++;
    }

    cout << endl;
    cout << "Mean execution time of a block: "<< mean(acc) * 1e-9 << " s" << endl;
    cout << "Standard deviation of execution time of a block: " << sqrt(variance(acc)) * 1e-9 << " s"<< endl;
    cout << "Standard deviation as percentage of mean: " << sqrt(variance(acc)) / mean(acc) * 100.0 << " %"<< endl;    
    
    return 0;
}
