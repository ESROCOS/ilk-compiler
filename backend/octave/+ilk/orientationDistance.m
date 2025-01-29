% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function d = orientationDistance(x_R_desired, x_R_actual)
    thresh = 1e-6;   % TODO %magic-number

    % TODO checks on the size 3x3
    R = x_R_actual' * x_R_desired;   %  this is 'actual_R_desired'

    x = R(3,2) - R(2,3);
    y = R(1,3) - R(3,1);
    z = R(2,1) - R(1,2);
    norm = sqrt(x*x + y*y + z*z);
    if norm < thresh
        d = ilk.AxisAngle([0.0, 0.0, 1.0], 0.0); % arbitrary choice of (0,0,1) axis
        return
    end

    theta = atan2( norm, trace(R)-1 );
    d = ilk.AxisAngle([x/norm, y/norm, z/norm], theta);
    % TODO check corner cases, theta close to 0/PI, bad numerical
