#! /usr/bin/env lua

ilk_compiler_version = "0.4.0"

local ilk_parser = require('ilk.parser')

local logger = require('log').new(
  "warning",
  require('log.writer.console.color').new()
)

local lfs = require "lfs"


local function generator(config)
  local genConfig = {
    path = config.path,
    sourceFileName = config.robotName
  }
  local generator = nil
  if config.backend == 'eigen' then
      generator = require('ilk.eigen')
      genConfig.headerFileName = config.robotName
  elseif config.backend == 'julia' then
      generator = require('ilk.julia')
  elseif config.backend == 'numpy' then
      generator = require('ilk.numpy')
  else
    error("Unknown backend '"..config.backend.."'")
  end
  return function(context, programs) generator(context, programs, genConfig) end
end



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
        if model_values[k] == nil then
          logger.warning("Could not find numeric values for pose " ..
                       k .. ", required by solver " .. programs[i].meta.solver_id)
        end
      end
    end
  end
  -- TODO check everything refers to the same robot, and it is consistent
  -- (e.g. same joint space size for each program)

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
  local sep = package.config:sub(1, 1) -- the path separator for the current OS
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
cli:flag("-v, --version", "prints the program's version and exits", printVersionAndExit)

-- Parse the command line arguments
--
local args, err = cli:parse(arg)
if not args and err then
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
    local chunk1, errmsg = loadfile(inputfolder .. ilk)
    if(chunk1==nil) then
      logger.error(errmsg)
    end
    local prog = chunk1()
    ilkSources[i] = prog
end
local model_values = {}
local chunk2, errmsg = loadfile(inputfolder .. 'model-constants.lua')
if(chunk2==nil) then
  errmsg = errmsg .. "\nThe generated code will lack the numerical constants of the robot model"
  logger.warning(errmsg)
else
  model_values = chunk2()
end

-- Parse the ilk sources, and create the internal representation of the solvers
-- model
local context,programs = parseInputs(ilkSources, model_values)


local common      = require("ilk.common")
local sanityCheck = require("ilk.sanity-checks")

for i,p in ipairs(programs) do
  sanityCheck(p, logger)
  p.metasignature = common.metaSignature(p)
end



-- Call the actual code generator
--
mkdir(args.OUTDIR)
local generator = generator(
                     {inputfolder=inputfolder,
                      path=args.OUTDIR,
                      backend=args.backend,
                      robotName=context.robotName} )
generator(context, programs)






local generate_metadata = false
if generate_metadata then
    local gen_metadata = require('ilk.eigen.gen-metadata')
    gen_metadata(args.OUTDIR, programs)
end
