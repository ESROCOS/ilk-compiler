--- See the docs of `ilk.backend.sample.backend-symbols`

local keys = require("ilk.parser").keys

local RET = {}

local backend_namespace = "ilk"

local strbits = {
  [keys.jointType.prismatic] = 'tr_z',
  [keys.jointType.revolute]  = 'rot_z',
  [keys.ctDir.a_x_b] = 'a_x_b',
  [keys.ctDir.b_x_a] = 'b_x_a'
}

local jointTransformSetvalue = function(program, jointPoseData)
    local func= strbits[jointPoseData.joint.kind] .. '__' .. strbits[jointPoseData.direction]
    local arg1= jointPoseData.statusVectVarName .. '(' .. jointPoseData.joint.coordinate+1 .. ')'
    local arg2= jointPoseData.transformName
    return arg2.. " = " .. backend_namespace .. "." .. func .."("..arg1..");"
end




local funcs = {
  lsSolve      = backend_namespace .. ".leastSquaresSolve",
  orientDist   = backend_namespace .. ".orientationDistance",
  linearCoords = backend_namespace .. ".linearCoords",
  angularCoords= backend_namespace .. ".angularCoords",
  ikPosDebug   = backend_namespace .. ".ikPosDebugStruct",
  ikPosConfig  = backend_namespace .. ".ikPosConfigStruct",
  gJacCol = {
    [keys.jointType.prismatic] = backend_namespace .. ".geometricJacobianColumn_prismatic",
    [keys.jointType.revolute]  = backend_namespace .. ".geometricJacobianColumn_revolute"
  },
  ct_twist = backend_namespace .. ".ctTwist"
}

local matrixT = function(r,c)
    return "zeros("..r..","..c..")"
end


local coordsID = {
  location   = {x="4",y="5",z="6"},
  orientation= {x="1",y="2",z="3"}
}
local coordsSpan = {
  location   = "4:6",
  orientation= "1:3",
}

local spatialVectorIndex = {
  [keys.jointType.prismatic] = coordsID.location.z,
  [keys.jointType.revolute]  = coordsID.orientation.z
}

local jointIndex = function(joint)
    return joint.coordinate+1
end

return {
    jointTransformSetvalue = jointTransformSetvalue,
    matrixT = matrixT,
    funcs = funcs,

    ik_pos_dbg = "ik_pos_dbg",
    ik_pos_cfg = "ik_pos_cfg",

    coordsID = coordsID,
    coordsSpan = coordsSpan,
    spatialVectorIndex = spatialVectorIndex,
    jointIndex = jointIndex,
}
