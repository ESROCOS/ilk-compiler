% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

function result = leastSquaresSolve(A, b)
    [U, S, V] = svd(A, "eco"); % A = U*S*V'
    result = V * (eye(size(S)) * 1./diag(S)) * U' * b;
end

% given the SVD decomposition
% A = U*S*V'
% then
% A% = V * S^(-1) * U'

%% eye(size(S)) * 1./diag(S)
%% gets the inverse of the diagonal matrix S (taking the reciprocal of each
%% element on the diagonal). I do not know if there is a better way to do the
%% same in Octave
