local tpl    = require("ilk.template-text").template_eval
local keys   = require('ilk.parser').keys
local common = require('ilk.common')
local langcom= require('ilk.langcommons')
local testcom= require('ilk.langcommons_test')
local lang   = require('ilk.julia.common')
local backend= require('ilk.julia.backend-symbols')


local FKDatasetComparisonTest = function(program, context, env)
  local tplText =
[[
@local datasetImport = "NaiveDataset"
import «backendModule»
import «robotModule»
import «datasetImport»
import LinearAlgebra

«solverArgs[1]» = «robotModule».ModelConstants

function test_fk_dataset( ds::«datasetImport».BinDataset )

    err_pos = 0;
    err_ori = 0;
    count   = 0;
@for i,arg in ipairs(dummy_args) do
    «dummyLocal(arg)»
@end
    while ! ds.eof
        «arg_q.name» = «datasetImport».readVector(ds, UInt16(«jss»));
    @for i,v in pairs(givenPoses) do
        «v» = «datasetImport».readPose(ds);
    @end
        «table.concat(oargs,",")» = «robotModule».«signature.toString({call=true, args=solverArgs})»;

    @for i = 1,outputPosesCount do
        Rdiff = «backendModule».«funcs.orientDist»(
                    «backendModule».«funcs.rotView»(«oPoses[i]»),
                    «backendModule».«funcs.rotView»(«givenPoses[i]»))
        err_ori += Rdiff.angle;
        err_pos += LinearAlgebra.norm(«backendModule».«funcs.posView»(«oPoses[i]»-«givenPoses[i]»));
        count = count + 1;
    @end
    end

    err_pos /= count
    err_ori /= count

    println("Average position error   : " * string(err_pos))
    println("Average orientation error: " * string(err_ori))
end

if abspath(PROGRAM_FILE) == @__FILE__
    if length(Base.ARGS) < 1
        print("Please provide a dataset file")
        exit(-1)
    end
    filename = Base.ARGS[1]
    ds = «datasetImport».openBinDataset( filename )

    test_fk_dataset( ds )
end
]]
  return common.tplEval_failOnError(tplText, env,  {verbose=true, xtendStyle=true})
end




local ik_test_fk__velocity = function(program, context, env)
  local tplText =
[[
import «backendModule»
import «robotModule»

«fkargs[1]» = «robotModule».ModelConstants

function test_ik_vel()
     «fkargs[2]» = rand(«jss»)

     fk_pose, fk_J = «robotModule».«fkSolver.signature.toString({call=true, args=fkargs})»

     qd = rand(«jss»)

     «twistVar» = fk_J * qd

     «table.concat(ikout, ',')» = «robotModule».«signature.toString({call=true, args=ikargs})»

     display( «twistVar» - fk_J * «ikout[1]» )
end

test_ik_vel();

]]
  return common.tplEval_failOnError(tplText, env,  {verbose=true, xtendStyle=true})
end


local ik_test_fk__position = function(program, context, env)
  local tplText =
[[
@local fkPose = "fkpose"

import «backendModule»
import «robotModule»
import LinearAlgebra

«fkargs[1]» = «robotModule».ModelConstants

function test_ik_pos(logstream)
    # Generate a random set point in the ee pose space
    «q» = rand(«jss»);
    q_rand = rand(«jss»);
    «qGuess» = «q» + q_rand/10;

    «fkPose», J = «robotModule».«fkSolver.signature.toString({call=true, args=fkargs})»;
    desp = «backendModule».«funcs.posView»(«fkPose»);
    deso = «backendModule».«funcs.rotView»(«fkPose»);

    # Call IK for position
    «ikcfg» = «backendModule».«bend.ik_pos_cfg»(0.004, 1e-6, 1e-6, 100, 1e-3);
    «table.concat(ikout, ',')» = «robotModule».«signature.toString({call=true, args=ikargs})»;

    prettyPrint = «backendModule».«matrix(6,4)»;

    prettyPrint[:,1] = «q»[:]
    prettyPrint[:,2] = «qGuess»[:]
    prettyPrint[:,3] = «qIK»[:]
    prettyPrint[:,4] = ( «q» - «qIK» );

    println("ee pos error:\t " * string( LinearAlgebra.norm(desp - «ikdbg».actual_pos) ) );
    println("ee or  error:\t " * string( «backendModule».«funcs.orientDist»(«ikdbg».actual_or, deso).angle ) );
    println("");
    println("q | q_guess | q_ik | q - q_ik");
    display(prettyPrint);
    println("");
    println("iterations count: " * string(«ikdbg».iter_count) );

end


octave = open("octave_log.m", "w")
test_ik_pos(octave)

]]
  return common.tplEval_failOnError(tplText, env,  {verbose=true, xtendStyle=true})
end



local juliaSpecifics = function(context, config)
  return {
    backend = backend,
    FKDatasetComparisonTest = FKDatasetComparisonTest,
    IKTestFKConsistency = {
      [keys.solverKind.ik.velocity] = ik_test_fk__velocity,
      [keys.solverKind.ik.position] = ik_test_fk__position
    },
    customizeCommonEnvironment = function(context, config, env)
      env.robotModule   = context.mainModule
      env.backendModule = context.backendModule
      env.matrix = function(r,c) return context.matrixInitExpr(r,c)  end
    end,
    linearCoordsViewCode = function(var)
      return context.backendModule .. "." .. backend.funcs.linearCoords .. "(" .. var .. ")"
    end,
    angularCoordsViewCode = function(var)
      return context.backendModule .. "." .. backend.funcs.angularCoords .. "(" .. var .. ")"
    end,
    defaultLocalDefinition = function(program, context, parameter) return lang.defaultLocalDefinition(program,context,parameter) end
  }
end

local Generator = function(context, config)
  return testcom.Generator(context, config, juliaSpecifics(context,config))
end

return Generator

