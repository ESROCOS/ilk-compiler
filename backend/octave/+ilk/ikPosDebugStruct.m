% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function dbg = ikPosDebugStruct()

dbg.iter_count = 0;
dbg.actual_pos = zeros(3,1);
dbg.actual_or = eye(3);
