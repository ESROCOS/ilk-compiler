# Copyright © 2024 Marco Frigerio
# Distributed under the 'BSD-2-Clause' license.
# See the LICENSE file for details.

function mx = tr_z__b_x_a(length)
    mx = eye(4);
    mx(3,4) = -length;
end
