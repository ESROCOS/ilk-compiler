local keys   = require('ilk.parser').keys
local common = require('ilk.common')
local langcom= require('ilk.langcommons')

local commonEnv = function(context, config, backend)
  return {
    bend  = backend,
    metat = common.metatypes,
    funcs = backend.funcs,
  }
end


local addCommonEnvForIK = function(env, program, context, config)
  env.fkSolver  = program.fkSolver
  env.fkargs = {}
  for i,arg in ipairs(program.fkSolver.signature.inputs) do
    table.insert( env.fkargs, arg.name )
  end

  env.ikargs = {}
  for i,arg in ipairs(program.signature.inputs) do
    table.insert( env.ikargs, arg.name )
  end

  env.ikout = {}
  for i,arg in ipairs(program.signature.outputs) do
    table.insert( env.ikout, arg.name )
  end

end


local FKDatasetComparisonTest = function(program, context, env, language)
  env.oargs = {}
  env.oPoses= {}
  env.givenPoses = {}
  local counter = 0
  for i,out in ipairs(program.signature.outputs) do
    local name = out.name
    if out.meta.metatype == common.metatypes.pose then
      counter = counter + 1
      table.insert(env.oPoses, out.name)
      table.insert(env.givenPoses, "given_"..counter)
    end
    table.insert(env.oargs, out.name)
  end
  env.outputPosesCount = counter
  env.solverArgs = {}
  for i,arg in ipairs(program.signature.inputs) do
    table.insert( env.solverArgs, arg.name )
  end
  -- Now mark the input arguments which are not required by the dataset-based
  -- test of FK. For example, at the moment we do not test velocities stuff,
  -- but if qd is in the argument list of the solver, the code must at least
  -- define a dummy for the test to compile/run
  env.dummy_args = {}
  for i,p in ipairs(program.signature.inputs) do
    if p.meta.metatype ~= common.metatypes.jointState and
       p.meta.metatype ~= common.metatypes.modelConsts then
      table.insert(env.dummy_args, p)
    end
  end
  env.dummyLocal = function(arg) return language.defaultLocalDefinition(program,context,arg) end
  return language.FKDatasetComparisonTest(program, context, env)
end


local IKTestFKConsistency = {}

IKTestFKConsistency[keys.solverKind.ik.velocity] =
function(program, context, env, language)
  env.twistVar = "twist"
  local cfgSpace = program.source.meta.solver_specs.configSpace
  local velArg = ""
  if cfgSpace == "pose" then
    velArg = env.twistVar
  elseif cfgSpace == "location" then
    velArg = language.linearCoordsViewCode( env.twistVar )
  elseif cfgSpace == "orientation" then
    velArg = language.angularCoordsViewCode( env.twistVar )
  end
  env.ikargs[1] = env.fkargs[1]
  env.ikargs[2] = env.fkargs[2]
  env.ikargs[3] = velArg
  return language.IKTestFKConsistency[keys.solverKind.ik.velocity](program, context, env)
end

IKTestFKConsistency[keys.solverKind.ik.position] =
function(program, context, env, language)
  local arg_fk_q = langcom.inputArgumentByType(program.fkSolver, common.metatypes.jointState)
  local arg_ik_q = langcom.inputArgumentByType(program, common.metatypes.jointState)
  env.q      = arg_fk_q.name
  env.qGuess = arg_ik_q.name
  env.qIK      = langcom.outputArgumentByType(program, common.metatypes.jointState).name
  env.ikcfg    = langcom.inputArgumentByType(program, common.metatypes.ikCfg).name
  env.ikdbg    = langcom.outputArgumentByType(program, common.metatypes.ikDbg).name
  return language.IKTestFKConsistency[keys.solverKind.ik.position](program, context, env)
end



local Generator = function(context, config, language)
  local env = commonEnv(context, config, language.backend)

  language.customizeCommonEnvironment(context, config, env)

  return function(program)
    env.arg_q = langcom.inputArgumentByType(program, common.metatypes.jointState)
    env.arg_mc= langcom.inputArgumentByType(program, common.metatypes.modelConsts)
      env.jss = program.source.meta.joint_space_size
    env.signature  = program.signature

    local numericComparisonTest = nil
    local fkConsistency = nil
    if program.source.meta.solver_type == keys.solverKind.fk then
      numericComparisonTest = FKDatasetComparisonTest
      fkConsistency = function() print("WARN: FK consistency test requested for a FK solver") end
    else
      addCommonEnvForIK(env, program, context, config)
      numericComparisonTest = function() return "" end
      fkConsistency = IKTestFKConsistency[program.source.meta.solver_type]
    end
    return {
       numericComparison = function() return numericComparisonTest(program, context, env, language) end,
       fkConsistency     = function() return fkConsistency(program, context, env, language) end
    }
  end
end

return {
  Generator = Generator
}
