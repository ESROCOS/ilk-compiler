--[[
   Main module for Python numpy backend support
   2018, KU Leuven, Belgium
   License: BSD 2-clause
--]]
local keys      = require('ilk.parser').keys
local comm      = require('ilk.common')
local numpycomm = require('ilk.numpy.common')
local sourcegen = require('ilk.numpy.source')

local testgen = require("ilk.numpy.test")
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




local function generator(ocontext, src_programs, config)

  -- Augment the context, with Python/Numpy specific context information :
  local context = {
    outer = ocontext,
    package = ocontext.robotName,
    mainModule = ocontext.robotName
  }

  -- Also augment the program model, with Python specific stuff
  local programs = {}
  for i, prog in ipairs( src_programs ) do
    local sign = numpycomm.signature(prog, context)
    programs[i] = {
      source=prog,
      signature=sign,
      poseValueExpression = poseValueExpression(prog, sign)
    }
  end
  context.programs = programs

  for i, prog in ipairs(programs) do
    if prog.source.meta.solver_type == keys.solverKind.ik.position or
       prog.source.meta.solver_type == keys.solverKind.ik.velocity then
      local fkid = prog.source.meta.solver_specs.fkSolverID
      local fkSolver = comm.findProgramByID(fkid, context)
      if fkSolver == nil then
        numpycomm.logger.warning("Could not find the parsed model of FK solver "..fkid..
            ", required by IK solver "..prog.source.meta.solver_id)
      end
      prog.fkSolver = fkSolver
    end
  end



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


  local mycfg = {
    importBackendAs = "backend"
  }

  -- Source files generation; main file and tests
  --
  local opath = config.path.."/"..context.package
  lfs.mkdir(opath)
  local fd = io.open(opath.."/"..config.sourceFileName..".py", "w") or io.stdout

  fd:write( sourcegen.heading(context, mycfg) )

  local testsGenerator = testgen(context, mycfg)
  local sourcetext
  local testtext
  local solverTestsGenerator
  for i,prog in ipairs(programs) do
    solverTestsGenerator = testsGenerator(prog)
    if prog.source.meta.solver_type == keys.solverKind.fk then

      sourcetext = sourcegen.fksource(prog, context, mycfg)
      fd:write(sourcetext)

      testtext = solverTestsGenerator.numericComparison()
      genTestFile(prog, testtext)
    else
      sourcetext = sourcegen.iksource(prog, context, mycfg)
      fd:write(sourcetext)

      testtext = solverTestsGenerator.fkConsistency()
      genTestFile(prog, testtext)
    end
    fd:write("\n\n")
  end
  fd:close()



end

return generator