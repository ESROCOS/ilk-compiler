local tpl    = require("template-text").template_eval
local keys   = require('ilk.parser').keys
local common = require('ilk.common')
local langcom= require('ilk.backend.common.common')
local cppcom = require('ilk.backend.eigen.common')
local backend= require('ilk.backend.eigen.backend-symbols')

local commonEnv = function(context, config)
  return {
    bend  = backend,
    funcs = backend.funcs,
    metat = common.metatypes,
    beNS  = cppcom.backendNS.qualifier(),
    robNS = context.namespace.qualifier(),
    header= config.headerFileName,
    valueType = function(param) return param.valuetype({nsQualified=true}, context) end,
    typ       = function(id) return cppcom.typeString(id, {nsQualified=true}, context) end
  }
end




local simpleMainTest = function(program, context, env, signatureUtils)
  env.localVars  = signatureUtils.localVarForEachArgument()
  env.solverArgs = signatureUtils.allArgsList()
  local ok, res = tpl(
[[
#include <iostream>
#include <ilk/eigen/misc.h>
#include "«header».h"

static «beNS»::RoundTheDifference<double> rounder(0.1);

int main(int argc, char** argv)
{
    ${localVars}

    // Set your inputs here
    «arg_q.name».setZero();

    «robNS»::«signature.toString({call=true, args=solverArgs})»;

@for i,outp in ipairs(signature.outputs) do
    //std::cout << std::endl << «outp.name» << std::endl;
@end

    // Print the difference between quantities with something like
    // std::cout << «beNS»::roundedDifference(A,B, rounder) << std::endl;

    return 0;
}
]],  env, {verbose=true, xtendStyle=true})

  if not ok then
    cppcom.logger.error("In the evaluation of the text template for the simple test (C++): " .. res)
    error("Failure in text template evaluation. See the log for more details.")
  end
  return res
end




local gen_fk_test = function(program, context, env, signatureUtils)
  env.solverArgs = signatureUtils.allArgsList()
  env.localVars  = signatureUtils.localVarForEachArgument()
  env.oPoses= {}
  env.givenPoses = {}
  local counter = 0
  for i,out in ipairs(program.signature.outputs) do
    if out.meta.metatype == common.metatypes.pose then
      counter = counter + 1
      table.insert(env.oPoses, out.name)
      table.insert(env.givenPoses, "given_"..counter)
    end
  end
  env.outputPosesCount = counter
  local ok, res = tpl(
[[
#include <iostream>

#include <ilk/eigen/core-types.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>
#include <ilk/eigen/test-utils.h>
#include "«header».h"

static «beNS»::RoundTheDifference<double> rounder(0.1);
static «localVars[1]»

int main(int argc, char** argv)
{
    if(argc < 2) {
        std::cerr << "Please provide a dataset file." << std::endl;
        return -1;
    }
    «beNS»::NaiveBinDataset data(argv[1]);

    «typ(metat.axisAngle)» Rdiff;
    double err_pos = 0;
    double err_ori = 0;
    int count = 0;

  @for i=2,#localVars do
    «localVars[i]»
  @end

@for i,v in pairs(givenPoses) do
    «typ(metat.pose)» «v»;
@end

    while( ! data.eof() )
    {
        data.readVector(«robNS»::dofs_count, «arg_q.name»);
    @for i,v in ipairs(givenPoses) do
        data.readPose(«v»);
    @end
        «robNS»::«signature.toString({call=true, args=solverArgs})»;

    @for i = 1,outputPosesCount do
        Rdiff = «beNS»::«funcs.orientDist»(
                    «beNS»::«funcs.rotView»(«oPoses[i]»),
                    «beNS»::«funcs.rotView»(«givenPoses[i]»));
        err_ori += Rdiff.angle;
        err_pos += («beNS»::«funcs.posView»(«oPoses[i]»-«givenPoses[i]»)).norm();
        count++;
    @end
    }

    err_pos /= count;
    err_ori /= count;

    std::cout << "Average position error   : " << err_pos << std::endl;
    std::cout << "Average orientation error: " << err_ori << std::endl;

    return 0;
}
]],  env, {verbose=true, xtendStyle=true})

  if not ok then error(res) end
  return res
end



local gen_ik_test_position = function(program, context, env, signatureUtils)

  local fkSignatureUtils = cppcom.signatureUtils(program.fkSolver, context)
  env.fkSolver = program.fkSolver
  env.fkargs   = fkSignatureUtils.allArgsList()
  env.fkLocals = fkSignatureUtils.localVarForEachArgument()
  env.fk_q     = langcom.inputArgumentByType(program.fkSolver, common.metatypes.jointState)
  env.fkPose   = langcom.outputArgumentByType(program.fkSolver, common.metatypes.pose)
  env.arg_J    = langcom.outputArgumentByType(program.fkSolver, common.metatypes.jacobian)
  env.ikargs = {
    [1] = env.fkargs[1],
    [2] = "ikcfg"
  }
  local cfgSpace = program.source.meta.solver_specs.configSpace
  if cfgSpace == "pose" then
    table.insert(env.ikargs, "desp")
    table.insert(env.ikargs, "deso")
  elseif cfgSpace == "location" then
    table.insert(env.ikargs, "desp")
  elseif cfgSpace == "orientation" then
    table.insert(env.ikargs, "deso")
  end
  table.insert(env.ikargs, "q_guess")
  for i,arg in ipairs(program.signature.outputs) do
     table.insert( env.ikargs, arg.name )
  end

  local ok, res = tpl(
[[
@local qGuess = ikargs[signature.meta.iParsCount]
@local qIK    = ikargs[signature.meta.iParsCount+1] -- first output argument
@local ikdbg  = ikargs[signature.meta.iParsCount+2]
#include <iostream>
#include <fstream>

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include "«header».h"

using namespace std;

static «beNS»::RoundTheDifference<double> rounder(0.1);
static «fkLocals[1]»

void test_ik_pos(std::ofstream& log)
{
  @for i=2,#fkLocals do
    «fkLocals[i]»
  @end
    «typ(metat.jointState)» «qGuess», q_rand, «qIK»;

    // Generate a random set point in the ee pose space
    «fk_q.name».setRandom();
    q_rand.setRandom();
    «qGuess» = q + q_rand/10;

    «robNS»::«fkSolver.signature.toString({call=true, args=fkargs})»;
    «typ(metat.position3d)» desp = «beNS»::«funcs.posView»(«fkPose.name»);
    «typ(metat.orient3d)»   deso = «beNS»::«funcs.rotView»(«fkPose.name»);

    // Call IK for position
    «typ(metat.ikCfg)» «ikargs[2]»;
    «typ(metat.ikDbg)» «ikdbg»;
    «ikargs[2]».max_iter = 500;
    «ikargs[2]».eps_or_err_norm = 1e-3;
    «ikargs[2]».ls_damping = 0.08;
    «robNS»::«signature.toString({call=true, args=ikargs})»;

    «beNS»::«bend.matrixT(robNS.."::dofs_count",4)» prettyPrint;

    prettyPrint.col(0) = «fk_q.name»;
    prettyPrint.col(1) = «qGuess»;
    prettyPrint.col(2) = «qIK»;
    prettyPrint.col(3) = «beNS»::roundedDifference( «fk_q.name», «qIK», rounder );

    cout << "ee pos error:\t" << «beNS»::roundedDifference(desp, «ikdbg».actual_pos, rounder).transpose() << endl;
    cout << "ee or  error:\t" << «beNS»::«funcs.orientDist»(«ikdbg».actual_or, deso).angle << endl << endl;
    cout << "q | q_guess | q_ik | q - q_ik" << endl;
    cout << prettyPrint << endl;

    cout << endl << "iterations count: " << «ikdbg».iter_count << endl;

    log << "ik.q_des   = [" << «fk_q.name».transpose() << "];" << endl;
    log << "ik.q_guess = [" << «qGuess».transpose() << "];" << endl;
    log << "ik.q_found = [" << «qIK».transpose() << "];" << endl;
}

int main()
{
    std::ofstream octave("octave_log.m");
    std::srand((unsigned int) time(0));
    test_ik_pos(octave);

    return 0;
}
]], env, {verbose=true, xtendStyle=true})
  if not ok then error(res) end
  return res
end

--config, idx, robot_name, solver_name, ik,
local gen_ik_test_velocity = function(program, context, env, signatureUtils)
  local fkSignatureUtils = cppcom.signatureUtils(program.fkSolver, context)
  env.fkSolver = program.fkSolver
  env.fkargs   = fkSignatureUtils.allArgsList()
  env.fkLocals = fkSignatureUtils.localVarForEachArgument()
  env.arg_J    = langcom.outputArgumentByType(program.fkSolver, common.metatypes.jacobian)
  env.twistVar = "twist"
  local cfgSpace = program.source.meta.solver_specs.configSpace
  local velArg = env.beNS.."::"
  if cfgSpace == "pose" then
    velArg = env.twistVar
  elseif cfgSpace == "location" then
    velArg = velArg..backend.funcs.linearCoords.."("..env.twistVar..")"
  elseif cfgSpace == "orientation" then
    velArg = velArg..backend.funcs.angularCoords.."("..env.twistVar..")"
  end
  env.ikargs = {
      [1] = env.fkargs[1],
      [2] = env.fkargs[2],
      [3] = velArg,
      [4] = "qd_ik"
  }
  local ok, res = tpl(
[[
#include <iostream>
#include <fstream>

#include <ilk/eigen/gjac.h>
#include <ilk/eigen/ik.h>
#include <ilk/eigen/misc.h>
#include <ilk/eigen/rots.h>

#include "«header».h"

static «beNS»::RoundTheDifference<double> rounder(0.1);
static «fkLocals[1]»

void test_ik_vel()
{
  @for i=2,#fkLocals do
    «fkLocals[i]»
  @end

     «arg_q.name».setRandom();
     «robNS»::«fkSolver.signature.toString({call=true, args=fkargs})»;

     «typ(metat.jointState)» qd, «ikargs[4]»;
     qd.setRandom();

     «typ(metat.twist)» «twistVar» = «arg_J.name» * qd;

     «robNS»::«signature.toString({call=true, args=ikargs})»;

     std::cout << «beNS»::roundedDifference(«twistVar», «arg_J.name»*«ikargs[4]», rounder).transpose() << std::endl;
}

int main()
{
	test_ik_vel();
	return 0;
}
]], env, {verbose=true, xtendStyle=true})
  if not ok then error(res) end
  return res
end


local genIKTest = function(program, context, env, signatureUtils)
    if program.source.meta.solver_type == keys.solverKind.ik.position then
        return gen_ik_test_position(program, context, env, signatureUtils)
    elseif program.source.meta.solver_type == keys.solverKind.ik.velocity then
        return gen_ik_test_velocity(program, context, env, signatureUtils)
    end
    error("Unknown kind of IK solver")
end



local Generator = function(context, config)
  local timings = require("ilk.backend.eigen.test-timings")
  local env = commonEnv(context, config)

  return function(program)
    local signatureUtils = cppcom.signatureUtils(program, context)
    env.arg_q = langcom.inputArgumentByType(program, common.metatypes.jointState)
    env.signature  = program.signature

    local numericComparisonTest = nil
    local timingTest = nil
    local fkConsistency = nil
    if program.source.meta.solver_type == keys.solverKind.fk then
      numericComparisonTest = gen_fk_test
      timingTest = timings.fk
      fkConsistency = function() return "" end
    else
      numericComparisonTest = function() return "" end
      timingTest = function() return "" end
      fkConsistency = genIKTest
    end

    return {
      simpleMain        = function() return simpleMainTest       (program, context, env, signatureUtils) end,
      numericComparison = function() return numericComparisonTest(program, context, env, signatureUtils) end,
      timing            = function() return timingTest           (program, context, env, signatureUtils) end,
      fkConsistency     = function() return fkConsistency        (program, context, env, signatureUtils) end
    }
  end
end

local M = {
  genIKTest = genIKTest,
  templateCommonEnv = commonEnv,
  Generator = Generator
}
return M

