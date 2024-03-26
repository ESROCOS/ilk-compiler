--[[
   Main module for Python numpy backend support
   2018, KU Leuven, Belgium
   License: BSD 2-clause
--]]
local keys      = require('ilk.parser').keys
local comm      = require('ilk.common')
local numpycomm = require('ilk.backend.numpy.common')
local sourcegen = require('ilk.backend.numpy.source')

local testgen = require("ilk.backend.numpy.test")
local lfs = require('lfs')


local poseValueExpression = function(program, signature)
  local mc = nil
  for i,p in pairs(signature.inputs) do
    if p.meta.metatype==comm.metatypes.modelConsts then
      mc = p.name
    end
  end
  if mc==nil then error("Fatal, could not find the ModelConstants parameter in the signature of program "..program.source.meta.solver_id) end
  return
  function(poseid)
    if program.model_poses.constant[poseid] ~= nil then
      return mc .. "." .. poseid
    else
     return poseid
   end
 end
end


local function augmentContext(oContext, sourcePrograms)
  -- Augment the context, with Python/Numpy specific context information :
  local context = {
    outer = oContext,
    package = oContext.robotName,
    mainModule = oContext.robotName,
    pybackend = {
      package = "ilknumpy",
      modules = {
        core = "backend",
        dataset = "dataset"
      }
    }
  }

  -- Also augment the program model, with Python specific stuff
  local programs = {}
  for i, prog in ipairs( sourcePrograms ) do
    local sign = numpycomm.signature(prog, context)
    programs[i] = {
      source=prog,
      signature=sign,
      poseValueExpression = poseValueExpression(prog, sign)
    }
  end
  context.programs = programs
  return context, programs
end



local function getGenerator(opsHandlers, sourceTweakConfig)

local function generator(context, programs, config)
  local testPath = config.path
  lfs.mkdir(testPath)
  local testFiles = {}
  local function genTestFile(program, text)
    local testFileName = program.source.meta.solver_id -- same as the solver-ID
    table.insert(testFiles,testFileName)
    local fdtest = io.open(testPath.."/"..testFileName..".py", "w")
    fdtest:write( text )
    fdtest:close()
  end

  -- Source files generation; main file and tests
  --
  local opath = config.path.."/"..context.package
  lfs.mkdir(opath)
  local fd = io.open(opath.."/"..config.sourceFileName..".py", "w") or io.stdout

  fd:write( sourcegen.heading(context, sourceTweakConfig) )


  local sourcetext
  local testtext
  for i,prog in ipairs(programs) do
    if prog.source.meta.solver_type == keys.solverKind.fk then
      sourcetext = sourcegen.fksource(prog, context, opsHandlers, sourceTweakConfig)
      fd:write(sourcetext)
    else
      sourcetext = sourcegen.iksource(prog, context, sourceTweakConfig)
      fd:write(sourcetext)
    end
    fd:write("\n\n")
  end
  fd:close()

  local testsGenerator = testgen(context, sourceTweakConfig)
  local solverTestsGenerator
  for i,prog in ipairs(programs) do
    solverTestsGenerator = testsGenerator(prog)
    if prog.source.meta.solver_type == keys.solverKind.fk then
      testtext = solverTestsGenerator.numericComparison()
      genTestFile(prog, testtext)
    else
      testtext = solverTestsGenerator.fkConsistency()
      genTestFile(prog, testtext)
    end
  end
end

  return generator
end

local function getGenerators(context, sourceTweakConfig)
  local opsHandlers = require("ilk.backend.numpy.ops").closuresOnConfig(sourceTweakConfig)
  local generator = getGenerator(opsHandlers, sourceTweakConfig)

  return opsHandlers, generator
end


return {
  augmentContext = augmentContext,
  getGenerators  = getGenerators
}
