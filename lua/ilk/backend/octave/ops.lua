local keys = require('ilk.parser').keys

local thisBackendCommons = require('ilk.backend.octave.common')
local thisBackendMetaAPI = require('ilk.backend.octave.backend-symbols')

local function tpleval(tpl, env)
    return thisBackendCommons.loadAndEvalTemplate(tpl, env, true)
end

local jacColTemplate = {
    [keys.jointType.revolute] = [[
«J.jac»(:,«J.col + 1») = «backend.funcs.gJacCol[jtype]»(
    poi_«J.jac»,
    «jpose»(1:3,4),
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


local ret = {
    poseCompose = poseCompose,
    geometricJacobianInit = geometricJacobianInit,
    geometricJacobianColumn = geometricJacobianColumn,
}


local niy = thisBackendCommons.loadTemplate(">>>>Not Implemented Yet<<<<", {})

setmetatable(ret, {__index=function() return function() return niy end end} )


return ret
