local keys = require("ilk.parser").keys

local RET = { namespaces = {"kul"} }

local strbits = {
  [keys.jointType.prismatic] = 'tr_z',
  [keys.jointType.revolute]  = 'rot_z',
  [keys.ctDir.a_x_b] = 'a_x_b',
  [keys.ctDir.b_x_a] = 'b_x_a'
}

RET.jointTransformSetvalue = function(program, jointPoseData)
  local func= strbits[jointPoseData.joint.kind] .. '__' .. strbits[jointPoseData.direction]
  local arg1= jointPoseData.statusVectVarName .. '(' .. jointPoseData.joint.coordinate .. ')'
  local arg2= jointPoseData.transformName
  return func .. "(" .. arg1 .. ", " .. arg2 .. ");"
end

--local function gJacColumn(gjac_col_op)
--  local metacall = {}
--  metacall.fname = "geometricJacobianColumn_" .. gjac_col_op.jtype
--
--  if gjac_col_op.jtype == 'revolute' then
--    metacall.args = nil
--  end
--end

local metat = require("ilk.common").metatypes
RET.types = {
  [metat.pose]        = "pose_t",
  [metat.position3d]  = "position_t",
  [metat.orient3d   ] = "rot_m_t",
  [metat.twist      ] = "twist_t",
  [metat.linVel3d   ] = "vector3_t",
  [metat.angVel3d   ] = "vector3_t",
  [metat.axisAngle  ] = "AxisAngle",
  [metat.ikCfg      ] = "ik_pos_cfg",
  [metat.ikDbg      ] = "ik_pos_dbg"
}

RET.funcs = {
  posView = "eg_get_position",
  rotView = "eg_get_rotation",
  zAxisView= "eg_get_zaxis",
  setPosition="eg_set_position",
  setRotation="eg_set_rotation",
  lsSolve="leastSquaresSolve",
  orientDist="orientationDistance",
  linearCoords="linearCoords",
  angularCoords="angularCoords",
  ct_twist="ct_twist",
  Sdot = {
    [keys.jointType.prismatic] = 'Sdot_prismatic',
    [keys.jointType.revolute]  = 'Sdot_revolute'
  }
}

RET.matrixT = function(r,c)
  return "Matrix<"..r..","..c..">"
end

RET.coordsID = {
  location   = {x="LX",y="LY",z="LZ"},
  orientation= {x="AX",y="AY",z="AZ"}
}

RET.spatialVectorIndex = {
  [keys.jointType.prismatic] = 'LZ',
  [keys.jointType.revolute]  = 'AZ',
}

return RET