local tpl    = require("ilk.template-text").template_eval
local common = require('ilk.common')
local cppcom = require('ilk.backend.eigen.common')
local testcom= require("ilk.backend.eigen.test")


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


local _,timing_headers = tpl(
[[
#include <iostream>
#include <fstream>

#include <time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;
]], {}, {verbose=true, xtendStyle=true, returnTable=true})



local _,timing_args = tpl(
[[
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
]], {}, {verbose=true, xtendStyle=true, returnTable=true})




local _,timing_reporting = tpl(
[[
cout << endl;
cout << "Mean execution time of a block: "<< mean(acc) * 1e-9 << " s" << endl;
cout << "Standard deviation of execution time of a block: " << sqrt(variance(acc)) * 1e-9 << " s"<< endl;
cout << "Standard deviation as percentage of mean: " << sqrt(variance(acc)) / mean(acc) * 100.0 << " %"<< endl;
]], {}, {verbose=true, xtendStyle=true, returnTable=true})




local _,time_delta_vars = tpl(
[[
struct timespec t_before;
struct timespec t_after;
double t_delta;
]], {}, {verbose=true, xtendStyle=true, returnTable=true})



local _,time_delta_calculations = tpl(
[[
double before_in_ns = t_before.tv_nsec + t_before.tv_sec * 1e9;
double after_in_ns = t_after.tv_nsec + t_after.tv_sec * 1e9;
t_delta = after_in_ns - before_in_ns;
acc(t_delta);
]], {}, {verbose=true, xtendStyle=true, returnTable=true})




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


local gen_fk_timing = function(program, context, env, signatureUtils)
  env.localVars  = signatureUtils.localVarForEachArgument()
  env.solverArgs = signatureUtils.allArgsList()
  env.clock_call = clock_call
  env.timing_headers = timing_headers
  env.timing_args    = timing_args
  env.timing_reporting = timing_reporting
  env.time_delta_vars = time_delta_vars
  env.time_delta_calculations = time_delta_calculations
  env.different_dataset_repetitions_default = "10"
  env.solver_call_repetitions_per_dataset_default = "10000000"
  local ok, res = tpl(
[[
${timing_headers}

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "«header».h"

using namespace std;

static «beNS»::RoundTheDifference<double> rounder(0.1);

int main(int argc, char** argv)
{
    int different_dataset_repetitions = «different_dataset_repetitions_default»;
    int solver_call_repetitions_per_dataset = «solver_call_repetitions_per_dataset_default»;
    ${timing_args}

    «typ(metat.axisAngle)» Rdiff;
    int count = 0;

    accumulator_set<double, stats<tag::variance> > acc;

    ${localVars}

    for (int i = 0; i < different_dataset_repetitions; i++)
    {
        «arg_q.name».setRandom();

        ${time_delta_vars}
        «clock_call("t_before")»

        for (int j = 0; j < solver_call_repetitions_per_dataset; j++) {
            «robNS»::«signature.toString({call=true, args=solverArgs})»;
        }
        «clock_call("t_after")»
        ${time_delta_calculations}
        count++;
    }
    ${timing_reporting}

    return 0;
}
]], env, {verbose=true, xtendStyle=true})

    if not ok then error(res) end
    return res
end


local gen_ik_timing = function(config, idx, robot_name, solver_name, ik)
    if ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.position then
        gen_ik_timing_position(config, idx, robot_name, solver_name, ik)
    elseif ik.kind == ilk_keywords.op_argument.inverse_kinematics.kind.velocity then
        gen_ik_timing_velocity(config, idx, robot_name, solver_name, ik)
    end
end


return { fk = gen_fk_timing }
