% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function column = geometricJacobianColumn_revolute(poi, jointOrigin, jointAxis)
    column( ilk.angularCoords() ) = jointAxis;
    column( ilk.linearCoords() ) = cross(jointAxis, poi - jointOrigin);
end
