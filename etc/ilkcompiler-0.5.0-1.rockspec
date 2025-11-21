package = "ilkcompiler"
version = "0.5.0-1"
source = {
  url    = "git@github.com:ESROCOS/ilk-compiler.git",
  branch = "master"
}
description = {
  summary  = "Compiler for robotics solvers in ILK format",
  detailed = [[
    The ILK-compiler transforms kinematics/dynamics solvers for articulated
    robots, written in the ILK format, into code of regular programming languages.
  ]],
  license = "BSD 2-clause",
  homepage = "https://github.com/ESROCOS/ilk-compiler"
}
dependencies = {
  "lua > 5.1",
  "luafilesystem",
  "lyaml",
  "lua_cliargs",
  "lua-log",
  "template-text >= 0.2.0",
}
build = {
  type = "builtin",
  modules = {}, -- everything in the "lua/" folder is picked by default
  install = {
    bin = {"lua/ilk-compiler.lua"}
  },
}
