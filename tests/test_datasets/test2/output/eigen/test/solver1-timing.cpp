#include <iostream>
#include <fstream>

#include <time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "ur5.h"

using namespace std;

static kul::RoundTheDifference<double> rounder(0.1);

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

    kul::AxisAngle Rdiff;
    int count = 0;

    accumulator_set<double, stats<tag::variance> > acc;

    ur5::ModelConstants mc;
    ur5::joint_state q;
    ur5::joint_state qd;
    kul::pose_t wrist_3__base;
    kul::twist_t v__wrist_3__base;
    ur5::Jacobian_t J_wrist_3_base;

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
        q.setRandom();

        struct timespec t_before;
        struct timespec t_after;
        double t_delta;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_before);

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
            ur5::solver1(mc, q, qd, wrist_3__base, v__wrist_3__base, J_wrist_3_base);
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