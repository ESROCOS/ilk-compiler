#! /usr/bin/env lua

ilk_compiler_version = "0.4.1"

local ilk_parser = require('ilk.parser')
local sep = package.config:sub(1, 1) -- the path separator for the current OS

local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

local lfs = require "lfs"


local function getBackend(backendName, config)
  local genConfig = {
    path = config.path,
    sourceFileName = config.robotName,
    codeTweaks = {}
  }
  local backend = nil
  if backendName == 'eigen' then
      backend = require('ilk.backend.eigen')
      genConfig.headerFileName = config.robotName
      genConfig.dumpMetaData = true
  elseif backendName == 'julia' then
      backend = require('ilk.backend.julia')
  elseif backendName == 'numpy' then
      backend = require('ilk.backend.numpy')
      genConfig.codeTweaks.importBackendAs = "backend"
  else
    error("Unknown backend '"..backendName.."'")
  end
  return backend, genConfig
end


local common = require("ilk.common")
-- Check the given ILK models and transforms each one into the internal representation.
-- This function also creates the 'context' object, which contains common data
-- for the current session, like the robot name and its numerical constants.

-- @return the context and the list of solver-models, in the internal format
--
local parseInputs = function(ilkSources, model_values)
  local programs = {}
  for i,source in ipairs(ilkSources) do
    programs[i] = ilk_parser.parse(source)
    if programs[i].model_poses ~= nil then
      for k,v in pairs(programs[i].model_poses.constant) do
        if model_values.poses[k] == nil then
          logger.warning("Could not find numeric values for pose " ..
                       k .. ", required by solver " .. programs[i].meta.solver_id)
        end
      end
    end
    programs[i].metasignature = common.metaSignature(programs[i])
  end
  -- TODO check everything refers to the same robot, and it is consistent
  -- (e.g. same joint space size for each program)

  --- The context data associated with a run of the ilk-compiler
  -- @table context
  -- @field robotName name of the robot model as appearing in the first ILK
  --   source program
  -- @field jointSpaceSize the size of the robot's joint space, also determined
  --   from the metadata of the first ILK source program
  -- @field modelValues the robot model constants, as given to the `parseInputs`
  --   function
  -- @field programs the list of parsed programs, one for each ILK source program
  local context = {
      robotName   = programs[1].meta.robot_name,
      jointSpaceSize = programs[1].meta.joint_space_size,
      modelValues = model_values,
      programs    = programs
  }
  return context,programs
end


-- Helper function to create a directory including parents
--
local mkdir = function(path)
  local pathStr = ""
  -- Check if the first character is the separator
  if (path:sub(1,1) == sep) then
    pathStr = sep
  end
  for dir in path:gmatch("[^" .. sep .. "]+") do
    pathStr = pathStr .. dir .. sep
    local attr = lfs.attributes(pathStr, "mode")
    if attr ~= "directory" then
      local rv = lfs.mkdir(pathStr)
      if rv==nil then
        logger.warning("Could not create output directory "..pathStr)
      end
    else
      logger.notice("Output directory "..pathStr.. " already exists")
    end
  end
end


local function printVersionAndExit()
  print("ILK-compiler version "..ilk_compiler_version)
  os.exit(0)
end

---
-- Program enters here
-----------------------------------
local cli = require "cliargs"

cli:set_name("ILK-compiler")
cli:argument("INDIR", "path to the directory with input files")
cli:argument("OUTDIR", "path to the directory where to place outputs")
cli:option("-b, --backend=BACKEND", "the language backed to use for code generation", "eigen")
cli:flag("-d, --dump-ops", "print to stdout the compiled operations, do not generate any file")
cli:flag("-s, --[no-]sanity", "do/skip consistency checks on the ILK sources", true)
cli:flag("-v, --version", "prints the program's version and exits", printVersionAndExit)

-- Parse the command line arguments
--
local args, err = cli:parse(arg)
if not args and err then
  print(err)
  cli:print_help()
  os.exit(1)
end


-- Read all the .ilk source files and execute them
--
local ilk_files = {}
local inputfolder = args.INDIR
for file in lfs.dir( inputfolder ) do
  local ext = file:match("^.+%.(.+)$")
  if ext == 'ilk' then
    ilk_files[#ilk_files+1] = file
  end
end
table.sort(ilk_files)


local ilkSources = {}
for i,ilk in ipairs(ilk_files) do
    local chunk1, errmsg = loadfile(inputfolder .. sep .. ilk)
    if(chunk1==nil) then
      logger.error(errmsg)
      os.exit(-1)
    end
    local prog = chunk1()
    ilkSources[i] = prog
end
local model_values = {}
local chunk2, errmsg = loadfile(inputfolder .. sep .. 'model-constants.lua')
if(chunk2==nil) then
  errmsg = errmsg .. "\nThe generated code will lack the numerical constants of the robot model"
  logger.warning(errmsg)
else
  model_values = chunk2()
end

-- Parse the ilk sources, and create the internal representation of the solvers
-- model
local context,programs = parseInputs(ilkSources, model_values)

if args.sanity then
  local sanityCheck = require("ilk.sanity-checks")
  for i,p in ipairs(programs) do
    sanityCheck(p, logger)
  end
end



-- Get the backend module and its specific configuration
--
local backend, config = getBackend(args.backend, {path=args.OUTDIR, robotName=context.robotName} )
context, programs = backend.augmentContext(context, programs)
local opsHandlers, generateFiles = backend.getGenerators(context, config.codeTweaks )

if args['dump-ops'] then
  local genericOpsHandlers = require("ilk.backend.common.ops-handlers")
  for i, program in pairs(programs) do
    for op, code in genericOpsHandlers.translate(program, opsHandlers) do
      local text = common.tplEval_failOnError("${code}", {code=code})
      print(text)
    end
  end
else
  mkdir(args.OUTDIR)
  generateFiles(context, programs, config)
end



