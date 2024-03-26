local keys = require("ilk.parser").keys

local RET = {}

local strbits = {
  [keys.jointType.prismatic] = 'tr_z',
  [keys.jointType.revolute]  = 'rot_z',
  [keys.ctDir.a_x_b] = 'a_x_b',
  [keys.ctDir.b_x_a] = 'b_x_a'
}

RET.jointTransformSetvalue = function(backend)
  return
  function(program, jointPoseData)
    local func= strbits[jointPoseData.joint.kind] .. '__' .. strbits[jointPoseData.direction]
    local arg1= jointPoseData.statusVectVarName .. '[' .. jointPoseData.joint.coordinate + 1 .. ']'
    local arg2= jointPoseData.transformName
    return arg2.. " = " .. backend .. "." .. func .."("..arg1..")"
  end
end



local metat = require("ilk.common").metatypes
RET.types = {
  [metat.pose]       = "pose_t",
  [metat.position3d] = "position_t",
  [metat.orient3d  ] = "rot_m_t",
  [metat.twist     ] = "twist_t",
  [metat.linVel3d  ] = "vector3_t",
  [metat.angVel3d  ] = "vector3_t",
  [metat.axisAngle ] = "AxisAngle",
  [metat.ikCfg     ] = "ik_pos_cfg",
  [metat.ikDbg     ] = "ik_pos_dbg"
}

RET.funcs = {
  posView  = "positionView",
  rotView  = "rotationView",
  zAxisView= "zaxisView",
  setPosition= "setPosition",
  setRotation= "setRotation",
  lsSolve    = "leastSquaresSolve",
  orientDist = "orientationDistance",
  linearCoords = "linearCoords",
  angularCoords= "angularCoords",
  gJacCol = {
    [keys.jointType.prismatic] = "geometricJacobianColumn_prismatic",
    [keys.jointType.revolute]  = "geometricJacobianColumn_revolute"
  },
  ct_twist="ct_twist",
  matrixInit = "matrix"
}



RET.ik_pos_dbg = "ik_pos_dbg"
RET.ik_pos_cfg = "ik_pos_cfg"

RET.coordsID = {
  location   = {x="4",y="5",z="6"},
  orientation= {x="1",y="2",z="3"}
}
RET.spatialVectorIndex = {
  [keys.jointType.prismatic] = RET.coordsID.location.z,
  [keys.jointType.revolute]  = RET.coordsID.orientation.z
}

return RET