local keys = require('ilk.parser').keys

local tplUtils           = require('ilk.backend.common.tpleval')
local thisBackendCommons = require('ilk.backend.octave.common')
local thisBackendMetaAPI = require('ilk.backend.octave.backend-symbols')

local function tpleval(tpl, env)
    return tplUtils.fail_on_error.load_eval(tpl, env, {returnTable=true})
end

local jacColTemplate = {
    [keys.jointType.revolute] = [[
«J.jac»(:,«J.col + 1») = «backend.funcs.gJacCol[jtype]»(...
    poi_«J.jac»,...
    «jpose»(1:3,4),...
    «jpose»(1:3,3));
]],
  [keys.jointType.prismatic] = "«J.jac»(:,«J.col + 1») = «backend.funcs.gJacCol[jtype]»(«jpose»(1:3,3));"
}

local poseCompose = function(program, poseComposeOp)
    local poseValueExpression = thisBackendCommons.poseValueExpressionGenerator(program)
    local ids = {
        result = poseComposeOp.res,
        arg1   = poseValueExpression(poseComposeOp.arg1),
        arg2   = poseValueExpression(poseComposeOp.arg2)
    }
    return tpleval("«result» = «arg2» * «arg1»;", ids)
end


local geometricJacobianInit = function(program, op, config)
    local poseValueExpression = thisBackendCommons.poseValueExpressionGenerator(program)
    local tplenv = {
        backend = thisBackendMetaAPI,
        poseValueExpression = poseValueExpression,
        op = op
    }
    return tpleval([[
«op.name» = «backend.matrixT(6, op.columns)»;
poi_«op.name» = «poseValueExpression(op.pose)»(1:3,4);
]], tplenv)
end

local geometricJacobianColumn = function(program, op, joint, config)
    local poseValueExpression = thisBackendCommons.poseValueExpressionGenerator(program)
    local codetpl = jacColTemplate[joint.kind]
    local tplenv = {
        backend  = thisBackendMetaAPI,
        jtype = joint.kind,
        jpose = poseValueExpression(op.joint_pose),
        J = op
    }
    return tpleval(codetpl, tplenv)
end


local jointVelocityTwist = function(program, jointVelOp, jointVelocitySpec, joint, qd_argument, config)
    local ids = {
        qd  = qd_argument.name,
        velocity = jointVelOp.arg,
        index   = thisBackendMetaAPI.jointIndex(joint),
        spatialIndex = thisBackendMetaAPI.spatialVectorIndex[ joint.kind ],
        funcs =  thisBackendMetaAPI.funcs,
        init = thisBackendMetaAPI.matrixT(6,1),
    }

    local texttpl = nil
    if jointVelocitySpec.polarity == -1 then
        ids.H = program.poseValueExpression(jointVelocitySpec.ctransform)
        texttpl =
[[«velocity» = «init»;
aux = «init»;
aux(«spatialIndex») = -«qd»(«index»);
«funcs.ct_twist»( «H», aux, «velocity» );
]]
  else
    texttpl =
[[«velocity» = «init»;
«velocity»(«spatialIndex») = «qd»(«index»);]]
  end
  return tpleval(texttpl, ids)
end


local jointVelComposeSnippets = function(program, velocityComposeOp, env, config)
    env.coordt    = thisBackendMetaAPI.funcs.ct_twist
    env.twistInit = thisBackendMetaAPI.matrixT(6,1)

    env.sptInd = thisBackendMetaAPI.spatialVectorIndex[ env.joint.kind ]
    env. qdInd = thisBackendMetaAPI.jointIndex(env.joint)

    return {
        initializeVelocityVariable = tpleval("«res» = «twistInit»;", env),
        subtractJointVelocity      = tpleval("«v2»(«sptInd») = «v2»(«sptInd») - «qd»(«qdInd»);", env),
        twistCoordinateTransform   = tpleval("«coordt»( «H», «v2», «res» );", env),
        addJointVelocity           = tpleval("«res»(«sptInd») = «res»(«sptInd») + «qd»(«qdInd»);", env)
    }
end


local ret = {
    poseCompose = poseCompose,
    geometricJacobianInit = geometricJacobianInit,
    geometricJacobianColumn = geometricJacobianColumn,
    jointVelocityTwist = jointVelocityTwist,
    jointVelComposeSnippets = jointVelComposeSnippets,
}


local niy = tpleval(">>>>Not Implemented Yet<<<<", {})

setmetatable(ret, {__index=function() return function() return niy end end} )


return ret
