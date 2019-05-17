module ILKBackend

using LinearAlgebra

MatrixType = Matrix{Float64}

function matrix(r,c)
    return zeros(Float64,r,c)
end

function pose()
    return MatrixType(I,4,4)
end

function setPosition(pose, x,y,z)
    pose[1,4] = x
    pose[2,4] = y
    pose[3,4] = z
end

function setRotation(pose, xx, xy, xz, yx, yy, yz, zx, zy, zz)
    pose[1,1] = xx
    pose[1,2] = xy
    pose[1,3] = xz
    pose[2,1] = yx
    pose[2,2] = yy
    pose[2,3] = yz
    pose[3,1] = zx
    pose[3,2] = zy
    pose[3,3] = zz
end

function positionView(pose)
    return view(pose, 1:3, 4)
end

function rotationView(pose)
    return view(pose, 1:3, 1:3)
end

function zaxisView(pose)
    return view(pose, 1:3, 3)
end

function rot_z__a_x_b(angle)
    mx = zeros(4,4)
    s = sin(angle)
    c = cos(angle)

    mx[1,1] = c
    mx[1,2] = -s
    mx[2,1] = s
    mx[2,2] = c
    mx[3,3] = 1
    mx[4,4] = 1
    return mx
end

function rot_z__b_x_a(angle)
    mx = zeros(4,4)
    s = sin(angle)
    c = cos(angle)

    mx[1,1] = c
    mx[1,2] = s
    mx[2,1] = -s
    mx[2,2] = c
    mx[3,3] = 1
    mx[4,4] = 1
    return mx
end


function tr_z__a_x_b(length)
    mx = pose()
    mx[3,4] = length
    return mx
end

function tr_z__b_x_a(length)
    mx = pose()
    mx[3,4] = -length
    return mx
end


function linearCoords(_6dvector)
    return view(_6dvector, 4:6, 1)
end

function angularCoords(_6dvector)
    return view(_6dvector, 1:3, 1)
end


function geometricJacobianColumn_revolute(poi, jointOrigin, jointAxis, column)
    angularCoords(column)[:] = jointAxis
     linearCoords(column)[:] = LinearAlgebra.cross(jointAxis, poi - jointOrigin )
end

function geometricJacobianColumn_prismatic(jointAxis, column)
    angularCoords(column)[:] = zeros(3,1)
     linearCoords(column)[:] = jointAxis
end


mutable struct ik_pos_dbg
    iter_count::UInt64
    actual_pos::MatrixType
    actual_or::MatrixType
end

struct ik_pos_cfg
    dt::Float64
    eps_pos_err_norm::Float64
    eps_or_err_norm::Float64
    max_iter::UInt64
    ls_damping::Float64
end


struct AxisAngle
    axis
    angle::Float64
end

function omega(aa::AxisAngle)
    return aa.axis*aa.angle
end


# The difference between two rotation matrices, as an axis-angle.
# @return the rotation required to go from the second to the first argument,
#         that is, the first "minus" the second.

function orientationDistance(_R_desired, _R_actual)
    thresh = 1e-6   # TODO #magic-number

    # TODO checks on the size 3x3
    R = transpose(_R_actual) *  _R_desired   #  this is 'actual_R_desired'

    x = R[3,2] - R[2,3]
    y = R[1,3] - R[3,1]
    z = R[2,1] - R[1,2]
    norm = sqrt(x*x + y*y + z*z)

    if norm < thresh
        return AxisAngle([0.0; 0.0; 1.0], 0.0) # arbitrary choice of (0,0,1) axis
    end

    theta = atan( norm, LinearAlgebra.tr(R)-1 )
    return AxisAngle([x/norm, y/norm, z/norm], theta)
    # TODO check corner cases, theta close to 0/PI, bad numerical
end


function leastSquaresSolve(A, b)
    fact = LinearAlgebra.svd(A)
    return (fact.V * diagm(0 => (1 ./ fact.S)) ) * transpose(fact.U) * b
end


function ct_twist(newframe_H_current, twist_current, twist_newframe)
    ## We need to perform a spatial motion vector coordinate transform, given
    ## the homogeneous coordinate transform for the same frames.
    ## Check e.g. chapter 2 of Roy's RBDA, for the relation between the two
    ## transforms, to understand the last line of this function

    R = rotationView( newframe_H_current );      # 3x3 rotation matrix
    w = angularCoords( twist_current );          # omega in the current coordinates
    angularCoords( twist_newframe )[:] = R * w;  # omega in the new coordinates
     linearCoords( twist_newframe )[:] = R * linearCoords( twist_current ) +
                           cross( positionView( newframe_H_current),
                                  angularCoords( twist_newframe )    );
end


end # module
