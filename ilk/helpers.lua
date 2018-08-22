local ansicolors = require('ansicolors')
local vlog = require('vlog')

--- helper, print a bright red errormsg
local function errmsg(...)
    print(ansicolors.bright(ansicolors.red(table.concat({ ... }, ' '))))
end

--- helper, print a yellow warning msg
local function warnmsg(...)
    print(ansicolors.yellow(table.concat({ ... }, ' ')))
end

--- helper, print a green sucess msg
local function succmsg(...)
    print(ansicolors.green(table.concat({ ... }, ' ')))
end


function usage()
    print([[
ilk-compiler: C/C++ compiler for kin-ir language

Usage: gen-offline [OPTIONS]
    -i          <input-file>     input language (.ilk)
    -b          <option>         backend option [eigen, gsl]
    -o          <filename>       output (generated C/C++ source)
    -m          <path>           generate makefile in <path> (optional)
    --indir     <path>           folder with all input files
                                    (.ilk + model-constants + robot-defs.h)
    --outdir    <folder>         all files generated in <folder> (optional),
                                 outdir override options [-m, -o]
    --compile   (NONE)           compile the output (optional)

    -h        prints this help
]])
end

local M = {}
M.errmsg = errmsg
M.warnmsg = warnmsg
M.succmsg = succmsg
M.usage = usage
return M


