local lfs  = require "lfs"
local psep = package.config:sub(1, 1) -- the path separator for the current OS

local outputBasePath = "/tmp/ilk-compiler-tests/out"



local function nextLine(file, lineCommentMatch)
  local linen = 0
  local line = nil
  local commentMatch = "^"..lineCommentMatch
  local removeSpacesPattern = "^%s*(.-)%s*$"

  local readline = function()
    local line = file:read()
    if line ~= nil then
      line = line:match(removeSpacesPattern)
    end
    linen = linen + 1
    return line
  end

  return function()
    line = readline()
    -- ignore blank lines, and those starting with a comment
    while line ~= nil do
      if line:match(commentMatch) or line == "" then
        line = readline()
      else
        return line, linen
      end
    end
  end
end

local function compare_files_content(filename, expected_folder_name, actual_folder_name, lineCommentMatch)
    local expected_file = io.open(expected_folder_name .. psep .. filename, 'r')
    local actual_file   = io.open(actual_folder_name .. psep .. filename, 'r')
    local nextExpected = nextLine(expected_file, lineCommentMatch)
    local nextActual   = nextLine(actual_file  , lineCommentMatch)
    local ne = 0
    local na = 0
    local mismatch = false
    local expected
    local actual
    repeat
      expected, ne = nextExpected()
      actual, na   = nextActual()
      mismatch = (expected ~= actual)
    until expected == nil or actual == nil or mismatch

    expected_file:close()
    actual_file:close()
    return mismatch, actual, na, expected, ne
end




local function file_exists(path, filename)
    local file = io.open(path .. psep .. filename, 'r')
    if file == nil then return false end
    file:close()
    return true
end



-- Creates a list of all files in the given directory, including subdirectories.
-- Each item of the list is the path of the file relative to the given base,
-- including the base itself
local function listFiles(dir,list)
  list = list or {} -- use provided list or create a new one

  for entry in lfs.dir(dir) do
    if entry ~= "." and entry ~= ".." then
      local fullPath = dir .. psep .. entry
      if lfs.attributes(fullPath).mode == 'directory' then
        listFiles(fullPath,list)
      else
        table.insert(list, fullPath)
      end
    end
  end

  return list
end

local function listSubDirectories(root, list)
  list = list or {} -- use provided list or create a new one

  for entry in lfs.dir(root) do
    if entry ~= "." and entry ~= ".." then
      if lfs.attributes(entry).mode == 'directory' then
        table.insert(list, entry)
      end
    end
  end
  return list
end

local lineCommentMatch = {
  eigen = "[//-/*-*/-*]",
  numpy = "#",
  julia = "#"
}



local function singleBackendTest(code, backend, filesToCompare, inputDir, outputDir, expectedOutputDir)
  return function()
      local passed = true
      print("\n------- Test " .. code .. ", backend: " .. backend .. " . . .\n")
      os.execute('rm -rf '..outputDir)
      os.execute('lua ilk-compiler.lua -b ' .. backend .. ' ' ..  inputDir .. psep .. ' ' .. outputDir.. psep)
      for i, file in ipairs(filesToCompare) do
          local result = ""
          if file_exists(outputDir, file) then
              local mismatch, actual, na, expected, ne = compare_files_content(file, expectedOutputDir, outputDir, lineCommentMatch[backend])
                  if mismatch then
                      if actual == nil then
                          na = "X"
                          actual = "[missing]"
                      end
                      result = file .. ": Mismatch found:\n" ..
                               "ACTUAL   (line " .. na .. "):  " .. actual ..
                             "\nEXPECTED (line " .. ne .. "):  " .. expected
                      passed = false
                      print(result .. "\n")
                  end
          else
              result = file .. " DOES NOT EXIST!"
              passed = false
              print(result .. "\n")
          end
      end
      if passed then
        print(". . . passed!\n")
      end
      return passed
  end
end

local function fileComparisonTests(code, inputDir, expectedOutputRoot)
  local tests = {}
  local subdirs = {}
  local currentDir = lfs.currentdir() -- this one will be an absolute path

  -- The folder with the expected generated files must have a subdirectory
  -- for each backend that needs to be tested; the subdirectory must be named
  -- as the backend itself.
  lfs.chdir(expectedOutputRoot)
  listSubDirectories(".", subdirs)
  --print(inputDir, expectedOutputRoot, table.concat(subdirs, " "))

  local count = 0
  for i,backendDir in ipairs(subdirs) do
    local outputDir         = outputBasePath .. psep .. code .. psep .. backendDir
    local expectedOutputDir = expectedOutputRoot .. psep .. backendDir

    -- Get the list of all the files which are expected to be generated
    lfs.chdir(currentDir)          -- necessary as the following are relative paths
    lfs.chdir(expectedOutputRoot)
    lfs.chdir(backendDir)
    local filesToCompare = listFiles(".")

    tests[backendDir] = singleBackendTest(code, backendDir, filesToCompare, inputDir, outputDir, expectedOutputDir)
    count = count + 1
  end
  lfs.chdir(currentDir)
  return tests, count
end

local M = {}
M.fileComparisonTests = fileComparisonTests
return M


