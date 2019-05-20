--[[
   Main module for Eigen backend support
   2018, KU Leuven, Belgium
   License: BSD 2-clause
--]]
local keys      = require('ilk.parser').keys
local comm      = require('ilk.common')
local cppcomm   = require('ilk.eigen.common')
local sourcegen = require('ilk.eigen.source')
local headergen = require('ilk.eigen.header')
local test      = require("ilk.eigen.test")
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

  -- Augment the context, with C++/Eigen specific context information :
  local context = {
    outer = ocontext,
    namespace = cppcomm.contextNS(ocontext)
  }

  local programs = {}
  for i, prog in ipairs( src_programs ) do
    -- Also augment the program model, with C++ specific stuff
    local sign = cppcomm.signature(prog, context)
    programs[i] = {
      source=prog,
      signature=sign,
      poseValueExpression = poseValueExpression(prog, sign)
    }
  end
  context.programs = programs

  if config.dumpMetaData then
    local misc = require("ilk.eigen.gen-metadata")
    misc.gen_metadata(context, config.path)
  end

  local testPath = config.path.."/test"
  lfs.mkdir(testPath)
  local testFiles = {}
  local function genTestFile(program, text, nameSuffix)
    local suffix = nameSuffix or ""
    local testFileName = program.source.meta.solver_id..suffix -- same as the solver-ID
    table.insert(testFiles,testFileName)
    local fdtest = io.open(testPath.."/"..testFileName..".cpp", "w")
    fdtest:write( text )
    fdtest:close()
  end

  local mycfg = {headerFileName=config.headerFileName}
  local testsGenerator = test.Generator(context, mycfg)

  --
  -- Source files generation
  --
  local fd = io.open(config.path.."/"..config.sourceFileName..".cpp", "w") or io.stdout
  local sourceHeading = sourcegen.heading(context, mycfg)
  fd:write(sourceHeading)

  local sourcetext
  local testtext
  for i,prog in ipairs(programs) do
    local solverTestsGenerator = testsGenerator( prog )
    testtext = solverTestsGenerator.simpleMain()
    genTestFile(prog, testtext)

    if prog.source.meta.solver_type == keys.solverKind.fk then
      sourcetext = sourcegen.fksource(prog, context)

      testtext = solverTestsGenerator.numericComparison()
      genTestFile(prog, testtext, "-cmp")
      testtext = solverTestsGenerator.timing()
      genTestFile(prog, testtext, "-timing")
    else
      sourcetext = sourcegen.iksource(prog, context)

      testtext = solverTestsGenerator.fkConsistency()
      genTestFile(prog, testtext, "-fk-consistency")
    end

    fd:write(sourcetext)
    fd:write("\n\n")
  end
  fd:close()

  --
  -- CMake file generation
  --
  local cmaketext = require('ilk.eigen.cmake').generator(
      { robot       = ocontext.robotName,
        libname     = ocontext.robotName .. 'kingen',
        files       = { libsource=config.sourceFileName,
                        libheader=config.headerFileName,
                        defsheader='robot-defs', test=testFiles},
        includePath = 'kingen/' .. ocontext.robotName
      } )

  fd = io.open(config.path.."/CMakeLists.txt", "w")
  fd:write(cmaketext)
  fd:close()

--  local make = require('ilk.eigen.makefile').gen_makefile
--  fd = io.open(config.path.."/makefile", "w")
--  make(fd, {testfiles=testFiles, libname=ocontext.robotName})
--  fd:close()

  --
  -- Header file generation
  --
  local header = headergen.generator(context, programs)
  fd = io.open(config.path.."/"..config.headerFileName..".h", "w") or io.stdout
  fd:write(header)
  fd:close()
  local robot_defs = headergen.robot_defs(context, programs)
  fd = io.open(config.path.."/robot-defs.h", "w") or io.stdout
  fd:write(robot_defs)
  fd:close()

end

return generator