local constants = require('ilk.constants')
local utils = require('utils')
local keywords = require('ilk.keywords')
local string = require('string')


local function test_header_eigen(config, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[
#include <iostream>
#include "$(filename).h"

]], { table = table, filename = config.filename })
    if not ok then error(res) end
    fd:write(res)
end

local function test_footer_eigen(config, idx)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local ok, res = utils.preproc([[

int main(int argc, char** argv) {
//  exec_query();
  return 0;
}
]], { table = table })
    if not ok then error(res) end
    fd:write(res)
end



local generate_pose_name = function(ik)
    return ik.target..'__wrt__'..ik.reference
end

local generate_rotation_name = function(ik)
    return ik.reference..'__R__'..ik.target
end

local clock_call = function(target)
    local text = "clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &"..target..");"
    return text
end

local timing_headers = function()
    local text = [[
#include <iostream>
#include <fstream>

#include <time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;
    ]]
    return text
end

local timing_parameters_extraction = function()
    local text = [[
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


    ]]
    return text
end

local timing_reporting = function()
    local text = [[
    cout << endl;
    cout << "Mean execution time of a block: "<< mean(acc) * 1e-9 << " s" << endl;
    cout << "Standard deviation of execution time of a block: " << sqrt(variance(acc)) * 1e-9 << " s"<< endl;
    cout << "Standard deviation as percentage of mean: " << sqrt(variance(acc)) / mean(acc) * 100.0 << " %"<< endl;    
    ]]
    return text
end

local time_delta_variables_declaration = function()
    local text = [[
        struct timespec t_before;
        struct timespec t_after;
        double t_delta;
    ]]
    return text
end

local time_delta_calculations = function()
    local text = [[
        double before_in_ns = t_before.tv_nsec + t_before.tv_sec * 1e9;
        double after_in_ns = t_after.tv_nsec + t_after.tv_sec * 1e9;
        t_delta = after_in_ns - before_in_ns;
        acc(t_delta);
    ]]
    return text
end

local gen_ik_timing_position = function(config, idx, robot_name, solver_name, ik)
    local fd = config.fd or io.stdout
    local idx = idx or 0

    local pos_temp_name = generate_pose_name(ik)
    local rot_temp_name = generate_rotation_name(ik)
    local compute_pos = false
    local compute_or = false
    local compute_params = ''
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        compute_pos = true
        compute_params = 'desp'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        compute_or = true
        compute_params = 'deso'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        compute_pos = true
        compute_or = true
        compute_params = 'desp, deso'
    end

    local solver_call = solver_name..'(kk, cfg, '..compute_params..', q_guess, q_ik, dbg);'


    local input_type = constants.backend_namespace..'::'..constants.pose_type


    local ok, res = utils.preproc([[
#include "$(robot_name).h"
#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
$(timing_headers)
using namespace std;
using namespace $(robot_name);
using namespace $(constants.backend_namespace);


static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

int main(int argc, char** argv)
{
    int different_dataset_repetitions = $(different_dataset_repetitions_default);
    int solver_call_repetitions_per_dataset = $(solver_call_repetitions_per_dataset_default);
$(timing_parameters_extraction)

    int count = 0;
    accumulator_set<double, stats<tag::variance> > acc;

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
    	$(constants.input_type) q, q_guess, q_rand, q_ik;

	    // Generate a random set point in the ee pose space
	    $(input_type) $(pos_temp_name);
	    $(constants.jacobian_type) J;
	    q.setRandom();
	    q_rand.setRandom();
	    q_guess = q + q_rand/10;

$(time_delta_variables_declaration)
        $(clock_call_before)

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
	        $(fk_name)(kk, q, $(pos_temp_name), J);
	        vector3_t desp = eg_get_position($(pos_temp_name));
	        rot_m_t   deso = eg_get_rotation($(pos_temp_name));

	        // Call IK for position
	        $(constants.backend_namespace)::ik_pos_cfg cfg;
	        $(constants.backend_namespace)::ik_pos_dbg dbg;
	        cfg.max_iter = 500;
	        cfg.eps_or_err_norm = 1e-3;
	        cfg.ls_damping = 0.08;
	        $(solver_call)
        }
        $(clock_call_after)

$(time_delta_calculations)

        count++;
    }

$(timing_reporting)
    return 0;
}

]], {
        table = table,
        pairs = pairs,
        timing_headers = timing_headers(),
        timing_parameters_extraction = timing_parameters_extraction(),
        timing_reporting = timing_reporting(),
        time_delta_calculations = time_delta_calculations(),
        time_delta_variables_declaration = time_delta_variables_declaration(),
        space = utils.gen_spaces('\t', idx),
        different_dataset_repetitions_default = "10",
        solver_call_repetitions_per_dataset_default = "10000000",
        compose = compose,
        clock_call_before = clock_call("t_before"),
        clock_call_after = clock_call("t_after"),
        input_type = input_type,
        solver_call = solver_call,
        pos_temp_name = pos_temp_name,
        robot_name = robot_name,
        solver_name = solver_name,
        vector_size = vector_size,
        fk_name = ik.fk,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const,
        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end


local gen_ik_timing_velocity = function(config, idx, robot_name, solver_name, ik)
    local fd = config.fd or io.stdout
    local idx = idx or 0


    local pos_temp_name = generate_pose_name(ik)
    local twist_def = ''
    local presentation_def = ''
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        twist_def = constants.backend_namespace..'::linearCoords(twist)'
        presentation_def = constants.backend_namespace..'::linearCoords('..constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder)).transpose()'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        twist_def = constants.backend_namespace..'::angularCoords(twist)'
        presentation_def = constants.backend_namespace..'::angularCoords('..constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder)).transpose()'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        twist_def = 'twist'
        presentation_def = constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder).transpose()'
    end

    local solver_call = solver_name..'(kk, q, '..twist_def..', qd_ik);'


    local input_type = constants.backend_namespace..'::'..constants.pose_type

    local ok, res = utils.preproc([[
#include "$(robot_name).h"

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
$(timing_headers)
using namespace std;
using namespace $(robot_name);
using namespace $(constants.backend_namespace);


static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

int main(int argc, char** argv)
{
    int different_dataset_repetitions = $(different_dataset_repetitions_default);
    int solver_call_repetitions_per_dataset = $(solver_call_repetitions_per_dataset_default);
$(timing_parameters_extraction)

    $(robot_name)::$(constants.input_type) q;
    $(constants.backend_namespace)::AxisAngle Rdiff;
    int count = 0;

    accumulator_set<double, stats<tag::variance> > acc;

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
        $(constants.input_type) q;
        $(input_type) $(pos_temp_name);
        $(constants.jacobian_type) J;
        $(constants.input_type) qd, qd_ik;
        qd.setRandom();
        q.setRandom();

$(time_delta_variables_declaration)
        $(clock_call_before)

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
             $(fk_name)(kk, q, $(pos_temp_name), J);
             vector6_t twist = J * qd;
             $(solver_call)
        }
        $(clock_call_after)

$(time_delta_calculations)

        count++;
    }

$(timing_reporting)
    return 0;
}
]], {
        table = table,
        pairs = pairs,
        timing_headers = timing_headers(),
        timing_parameters_extraction = timing_parameters_extraction(),
        timing_reporting = timing_reporting(),
        time_delta_calculations = time_delta_calculations(),
        time_delta_variables_declaration = time_delta_variables_declaration(),
        space = utils.gen_spaces('\t', idx),
        different_dataset_repetitions_default = "10",
        solver_call_repetitions_per_dataset_default = "10000000",
        compose = compose,
        clock_call_before = clock_call("t_before"),
        clock_call_after = clock_call("t_after"),
        input_type = input_type,
        solver_call = solver_call,
        presentation_def = presentation_def,
        pos_temp_name = pos_temp_name,
        robot_name = robot_name,
        solver_name = solver_name,
        vector_size = vector_size,
        fk_name = ik.fk,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const,
        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end


local gen_fk_timing = function(config, idx, robot_name, solver_name, olist)
    local fd = config.fd or io.stdout
    local idx = idx or 0
    local olist_no_jacobians = {}
    local olist_only_jacobians = {}
    for i,v in pairs(olist) do
        if string.match(v[1],constants.pose_type) then
            olist_no_jacobians[#olist_no_jacobians+1] = v
        else
            olist_only_jacobians[#olist_only_jacobians+1] = v
        end
    end


    local ok, res = utils.preproc([[
$(timing_headers)

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "$(robot_name).h"

using namespace std;
using namespace $(robot_name);

static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;



int main(int argc, char** argv)
{
    int different_dataset_repetitions = $(different_dataset_repetitions_default);
    int solver_call_repetitions_per_dataset = $(solver_call_repetitions_per_dataset_default);
    $(timing_parameters_extraction)

    $(robot_name)::$(constants.input_type) q;
    $(constants.backend_namespace)::AxisAngle Rdiff;
    int count = 0;

    accumulator_set<double, stats<tag::variance> > acc;

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
        q.setRandom();

$(time_delta_variables_declaration)
        $(clock_call_before)

@local counter = 0
@for i, v in pairs(olist_with_jacobians) do
@   counter = counter + 1
        $(v[1]) computed_$(counter);
@end

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
            $(solver_name)(kk, q,
@for i = 1,#olist_with_jacobians do
@   local comma = ","
@   if i == #olist_with_jacobians then
@       comma = ");"
@   end
                           computed_$(i)$(comma)
@end
        }
        $(clock_call_after)

$(time_delta_calculations)

        count++;
    }

$(timing_reporting)

    return 0;
}
]], {
        table = table,
        pairs = pairs,
        string = string,
        timing_headers = timing_headers(),
        timing_parameters_extraction = timing_parameters_extraction(),
        timing_reporting = timing_reporting(),
        time_delta_variables_declaration = time_delta_variables_declaration(),
        time_delta_calculations = time_delta_calculations(),
        different_dataset_repetitions_default = "10",
        solver_call_repetitions_per_dataset_default = "10000000",
        space = utils.gen_spaces('\t', idx),
        clock_call_before = clock_call("t_before"),
        clock_call_after = clock_call("t_after"),
        compose = compose,
        robot_name = robot_name,
        solver_name = solver_name,
        number_of_outputs = #olist,
        olist = olist_no_jacobians,
        olist_with_jacobians = olist,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const
    })
    if not ok then error(res) end
    fd:write(res)
end



local gen_fk_test = function(config, idx, robot_name, solver_name, olist)
    local fd = config.fd or io.stdout
    local idx = idx or 0
--    local compose = args.compose
--    local model_const = args.model_const
--    local olist = args.outputs
--    local identifiers = args.identifiers

    local olist_no_jacobians = {}
    local olist_only_jacobians = {}
    for i,v in pairs(olist) do
        if string.match(v[1],constants.pose_type) then
            olist_no_jacobians[#olist_no_jacobians+1] = v
        else
            olist_only_jacobians[#olist_only_jacobians+1] = v
        end
    end


    local ok, res = utils.preproc([[
#include <iostream>

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "$(robot_name).h"

using namespace std;
using namespace $(robot_name);

static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

int main(int argc, char** argv)
{
    if(argc < 2) {
        std::cerr << "Please provide a dataset file." << std::endl;
        return -1;
    }

    $(constants.backend_namespace)::TextDataset data(argv[1]);

    $(robot_name)::$(constants.input_type) q;
    $(constants.backend_namespace)::AxisAngle Rdiff;
    double err_pos = 0;
    double err_ori = 0;
    int count = 0;
    while( ! data.eof() )
    {
        data.readVector($(robot_name)::dofs_count, q);
@local counter = 0
@for i, v in pairs(olist_with_jacobians) do
@   counter = counter + 1
@   if string.match(v[1],constants.pose_type) then
        $(v[1]) given_$(counter), computed_$(counter);
        data.readPose(given_$(counter));
@   else
        $(v[1]) computed_$(counter);
@   end
@end
        $(solver_name)(kk, q,
@for i = 1,#olist_with_jacobians do
@   local comma = ","
@   if i == #olist_with_jacobians then
@       comma = ");"
@   end
                       computed_$(i)$(comma)
@end
@for i = 1,#olist do
        Rdiff = $(constants.backend_namespace)::orientationDistance(
            $(constants.backend_namespace)::eg_get_rotation(computed_$(i)),
            $(constants.backend_namespace)::eg_get_rotation(given_$(i)));
        err_ori += Rdiff.angle;
        err_pos += ($(constants.backend_namespace)::eg_get_position(computed_$(i)-given_$(i))).norm();
        count++;
@end
    }

    err_pos /= count;
    err_ori /= count;

    std::cout << "Average position error   : " << err_pos << std::endl;
    std::cout << "Average orientation error: " << err_ori << std::endl;

    return 0;
}
]], {
        table = table,
        pairs = pairs,
        string = string,
        space = utils.gen_spaces('\t', idx),
        compose = compose,
        robot_name = robot_name,
        solver_name = solver_name,
        number_of_outputs = #olist,
        olist = olist_no_jacobians,
        olist_with_jacobians = olist,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const
    })
    if not ok then error(res) end
    fd:write(res)
end

local gen_ik_test_position = function(config, idx, robot_name, solver_name, ik)
    local fd = config.fd or io.stdout
    local idx = idx or 0

    local pos_temp_name = generate_pose_name(ik)
    local rot_temp_name = generate_rotation_name(ik)
    local compute_pos = false
    local compute_or = false
    local compute_params = ''
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        compute_pos = true
        compute_params = 'desp'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        compute_or = true
        compute_params = 'deso'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        compute_pos = true
        compute_or = true
        compute_params = 'desp, deso'
    end

    local solver_call = solver_name..'(kk, cfg, '..compute_params..', q_guess, q_ik, dbg);'


    local input_type = constants.backend_namespace..'::'..constants.pose_type


    local ok, res = utils.preproc([[
#include "$(robot_name).h"

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace $(robot_name);
using namespace $(constants.backend_namespace);


static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

void test_ik_pos(std::ofstream& log)
{
	$(constants.input_type) q, q_guess, q_rand, q_ik;

	// Generate a random set point in the ee pose space
	$(input_type) $(pos_temp_name);
	$(constants.jacobian_type) J;
	q.setRandom();
	q_rand.setRandom();
	q_guess = q + q_rand/10;

	$(fk_name)(kk, q, $(pos_temp_name), J);
	vector3_t desp = eg_get_position($(pos_temp_name));
	rot_m_t   deso = eg_get_rotation($(pos_temp_name));

	// Call IK for position
	$(constants.backend_namespace)::ik_pos_cfg cfg;
	$(constants.backend_namespace)::ik_pos_dbg dbg;
	cfg.max_iter = 500;
	cfg.eps_or_err_norm = 1e-3;
	cfg.ls_damping = 0.08;
	$(solver_call)

	Matrix<6,4> prettyPrint;

	prettyPrint.col(0) = q;
	prettyPrint.col(1) = q_guess;
	prettyPrint.col(2) = q_ik;
	prettyPrint.col(3) = roundedDifference( q, q_ik, rounder );

	cout << "ee pos error:\t" << roundedDifference(desp, dbg.actual_pos, rounder).transpose() << endl;
	cout << "ee or  error:\t" << $(constants.backend_namespace)::orientationDistance(dbg.actual_or, deso).angle << endl << endl;
	cout << "q | q_guess | q_ik | q - q_ik" << endl;
	cout << prettyPrint << endl;

	cout << endl << "iterations count: " << dbg.iter_count << endl;

	log << "ik.q_des = [" << q.transpose() << "];" << endl;
	log << "ik.q_guess = [" << q_guess.transpose() << "];" << endl;
	log << "ik.q_found = [" << q_ik.transpose() << "];" << endl;
}

int main()
{
    std::ofstream octave("$(robot_name)_$(solver_name)_log.m");
	std::srand((unsigned int) time(0));
	test_ik_pos(octave);

	return 0;
}
]], {
        table = table,
        pairs = pairs,
        space = utils.gen_spaces('\t', idx),
        compose = compose,
        input_type = input_type,
        solver_call = solver_call,
        pos_temp_name = pos_temp_name,
        robot_name = robot_name,
        solver_name = solver_name,
        vector_size = vector_size,
        fk_name = ik.fk,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const,
        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end


local gen_ik_test_velocity = function(config, idx, robot_name, solver_name, ik)
    local fd = config.fd or io.stdout
    local idx = idx or 0


    local pos_temp_name = generate_pose_name(ik)
    local twist_def = ''
    local presentation_def = ''
    if ik.vectors == keywords.op_argument.inverse_kinematics.vector.linear then
        twist_def = constants.backend_namespace..'::linearCoords(twist)'
        presentation_def = constants.backend_namespace..'::linearCoords('..constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder)).transpose()'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.angular then
        twist_def = constants.backend_namespace..'::angularCoords(twist)'
        presentation_def = constants.backend_namespace..'::angularCoords('..constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder)).transpose()'
    elseif ik.vectors == keywords.op_argument.inverse_kinematics.vector.pose then
        twist_def = 'twist'
        presentation_def = constants.backend_namespace..'::roundedDifference(twist, J*qd_ik, rounder).transpose()'
    end

    local solver_call = solver_name..'(kk, q, '..twist_def..', qd_ik);'


    local input_type = constants.backend_namespace..'::'..constants.pose_type

    local ok, res = utils.preproc([[
#include "$(robot_name).h"

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace $(robot_name);
using namespace $(constants.backend_namespace);


static $(constants.backend_namespace)::RoundTheDifference<double> rounder(0.1);
static mc_config kk;

void test_ik_vel()
{
     $(constants.input_type) q;
     $(input_type) $(pos_temp_name);
     $(constants.jacobian_type) J;

     q.setRandom();
     $(fk_name)(kk, q, $(pos_temp_name), J);

     $(constants.input_type) qd, qd_ik;
     qd.setRandom();

     vector6_t twist = J * qd;

     $(solver_call)

     cout << $(presentation_def) << endl;
}


int main()
{
	test_ik_vel();
	return 0;
}
]], {
        table = table,
        pairs = pairs,
        space = utils.gen_spaces('\t', idx),
        compose = compose,
        input_type = input_type,
        solver_call = solver_call,
        presentation_def = presentation_def,
        pos_temp_name = pos_temp_name,
        robot_name = robot_name,
        solver_name = solver_name,
        vector_size = vector_size,
        fk_name = ik.fk,
        result_type = result_type,
        constants = constants,
        identifiers = identifiers,
        type_declaration_if_variable_was_not_yet_defined = type_declaration_if_variable_was_not_yet_defined,
        model_const = model_const,
        olist = olist
    })
    if not ok then error(res) end
    fd:write(res)
end

local gen_ik_test = function(config, idx, robot_name, solver_name, ik)
    if ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.position then
        gen_ik_test_position(config, idx, robot_name, solver_name, ik)
    elseif ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.velocity then
        gen_ik_test_velocity(config, idx, robot_name, solver_name, ik)
    end
end

local gen_ik_timing = function(config, idx, robot_name, solver_name, ik)
    if ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.position then
        gen_ik_timing_position(config, idx, robot_name, solver_name, ik)
    elseif ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.velocity then
        gen_ik_timing_velocity(config, idx, robot_name, solver_name, ik)
    end
end


local M = {}
M.gen_fk_test = gen_fk_test
M.gen_ik_test = gen_ik_test
M.gen_fk_test = gen_fk_test
M.gen_ik_timing = gen_ik_timing
M.gen_fk_timing = gen_fk_timing
M.header = test_header_eigen
M.footer = test_footer_eigen

return M
