local tpl    = require("ilk.template-text").template_eval
local keys   = require('ilk.parser').keys
local common = require('ilk.common')
local langcom= require('ilk.langcommons')
local testcom= require('ilk.langcommons_test')
local npcom  = require('ilk.numpy.common')
local backend= require('ilk.numpy.backend-symbols')


local FKDatasetComparisonTest = function(program, context, env)
  local tplText =
[[
from __future__ import print_function
import sys
import numpy as np

from ilknumpy import backend as «backendInport»
from ilknumpy import dataset
from «package» import «robotModule» as rob

«arg_mc.name» = rob.ModelConstants()

if __name__ == "__main__" :

    if len(sys.argv) < 2 :
        print("Please provide a dataset file", file=sys.stderr)
        exit(-1)

    data = dataset.NaiveBinDataset(sys.argv[1]);

    err_pos = 0;
    err_ori = 0;
    count   = 0;

@for i,arg in ipairs(dummy_args) do
    «dummyLocal(arg)»
@end

    while not data.eof :
        «arg_q.name» = data.readVector(«jss»)
        @for i,v in pairs(givenPoses) do
        «v» = data.readPose()
        @end
        «table.concat(oargs,",")» = rob.«signature.toString({call=true, args=solverArgs})»

    @for i = 1,outputPosesCount do
        Rdiff = «backendInport».«funcs.orientDist»(
                    «backendInport».«funcs.rotView»(«oPoses[i]»),
                    «backendInport».«funcs.rotView»(«givenPoses[i]»))
        err_ori += Rdiff.angle
        err_pos += np.linalg.norm(«backendInport».«funcs.posView»(«oPoses[i]»-«givenPoses[i]»))
        count = count + 1
    @end

    err_pos /= count
    err_ori /= count

    print("Average position error   : {0}".format(err_pos))
    print("Average orientation error: {0}".format(err_ori))


]]
  local text = common.tplEval_failOnError(tplText, env, {verbose=true, xtendStyle=true})
  return text
end


local ik_test_fk__velocity = function(program, context, env)
  local tplText =
[[
from __future__ import print_function
import sys
import numpy as np

from ilknumpy import backend as «backendInport»
from «package» import «robotModule» as rob

«arg_mc.name» = rob.ModelConstants()

def test_ik_vel() :
     «fkargs[2]» = np.random.random_sample(«jss»)

     fk_pose, fk_J = rob.«fkSolver.signature.toString({call=true, args=fkargs})»

     qd = np.random.random_sample(«jss»)

     «twistVar» = fk_J @ qd

     qd_ik = rob.«signature.toString({call=true, args=ikargs})»

     print( np.round( «twistVar» - fk_J @ qd_ik, decimals=5) )

if __name__ == "__main__" :
    test_ik_vel()
]]
  return common.tplEval_failOnError(tplText, env, {verbose=true, xtendStyle=true})
end


local ik_test_fk__position = function(program, context, env)
  local tplText =
[[
@local fkPose = "fkpose"
from __future__ import print_function
import sys, math
import numpy as np

from ilknumpy import backend as «backendInport»
from «package» import «robotModule» as rob

«arg_mc.name» = rob.ModelConstants()


def test_ik_pos(logstream) :
    # Generate a random set point in the ee pose space
    «q»    = np.random.rand(«jss»)
    q_rand = np.random.rand(«jss»)
    «qGuess» = «q» + q_rand/10

    «fkPose», J = rob.«fkSolver.signature.toString({call=true, args=fkargs})»
    desp = «backendInport».«funcs.posView»(«fkPose»)
    deso = «backendInport».«funcs.rotView»(«fkPose»)

    # Call IK for position
    «ikcfg» = «backendInport».«bend.ik_pos_cfg»(
        max_iter = 500,
        eps_or_err_norm = 1e-3)

    «table.concat(ikout, ',')» = rob.«signature.toString({call=true, args=ikargs})»

    prettyPrint = «backendInport».«bend.matrixT(6,4)»

    prettyPrint[:,0] = «q»[:]
    prettyPrint[:,1] = «qGuess»[:]
    prettyPrint[:,2] = «qIK»[:]
    prettyPrint[:,3] = (np.round( «q» - «qIK», decimals=5 ))[:]

    print("ee pos error:\t {0}".format( np.linalg.norm(desp - «ikdbg».actual_pos) ) )
    print("ee or  error:\t {0}".format( «backendInport».«funcs.orientDist»(«ikdbg».actual_or, deso).angle ) )
    print("")
    print("q | q_guess | q_ik | q - q_ik")
    print(prettyPrint)
    print("")
    print("iterations count: {0}".format(«ikdbg».iter_count) )

    #log << "ik.q_des   = [" << «fkargs[2]».transpose() << "];" << endl;
    #log << "ik.q_guess = [" << «qGuess».transpose() << "];" << endl;
    #log << "ik.q_found = [" << «qIK».transpose() << "];" << endl;


if __name__ == "__main__" :
    octave = open("octave_log.m", "w")
    #srand((unsigned int) time(0))
    test_ik_pos(octave)

]]
  return common.tplEval_failOnError(tplText,env,{verbose=true, xtendStyle=true})
end







local pythonSpecifics = function(context, config)
  return {
    backend = backend,
    FKDatasetComparisonTest = FKDatasetComparisonTest,
    IKTestFKConsistency = {
      [keys.solverKind.ik.velocity] = ik_test_fk__velocity,
      [keys.solverKind.ik.position] = ik_test_fk__position
    },
    customizeCommonEnvironment = function(context, config, env)
      env.robotModule = context.mainModule
      env.package = context.package
      env.backendInport = config.importBackendAs
    end,
    linearCoordsViewCode = function(var)
      return config.importBackendAs .. "." .. backend.funcs.linearCoords .. "(" .. var .. ")"
    end,
    angularCoordsViewCode = function(var)
      return config.importBackendAs .. "." .. backend.funcs.angularCoords .. "(" .. var .. ")"
    end,
    defaultLocalDefinition = function(program, context, parameter) return npcom.defaultLocalDefinition(program,context,parameter) end
  }
end

local Generator = function(context, config)
  return testcom.Generator(context, config, pythonSpecifics(context,config))
end


return Generator

