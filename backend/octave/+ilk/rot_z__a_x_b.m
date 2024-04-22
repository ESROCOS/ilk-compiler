% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function mx = rot_z__a_x_b(angle)
    mx = eye(4);
    s = sin(angle);
    c = cos(angle);
    mx(1,1) = c;
    mx(1,2) = -s;
    mx(2,1) = s;
    mx(2,2) = c;
end
