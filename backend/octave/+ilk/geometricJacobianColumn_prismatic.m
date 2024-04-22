% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function column = geometricJacobianColumn_prismatic(jointAxis)
    column( ilk.angularCoords() ) = zeros(3,1);
    column( ilk.linearCoords ) = jointAxis;
endfunction
