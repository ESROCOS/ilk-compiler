package = "ilkcompiler"
version = "0.4.0-1"
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
  "yaml",
  "lua_cliargs",
  "lua-log"
}
build = {
  type = "builtin",
}
