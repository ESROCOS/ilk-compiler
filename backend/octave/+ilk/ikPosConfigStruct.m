% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function cfg = ikPosConfigStruct()

cfg.dt = 0.004;
cfg.eps_pos_err_norm = 1e-3;
cfg.eps_or_err_norm = 1e-3;
cfg.max_iter = 300;
cfg.ls_damping = 0.05;
