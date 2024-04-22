% Copyright © 2024 Marco Frigerio
% Distributed under the 'BSD-2-Clause' license.
% See the LICENSE file for details.

classdef AxisAngle
    properties
        axis
        angle
    end
    methods
        function obj = AxisAngle(varargin)
            if nargin == 0
                obj.axis = [1.0; 0.0; 0.0];
                obj.angle = 0.0;
            else
                axis = (varargin{1} / norm(varargin{1}));
                obj.axis = axis(:); % (:) to force a column vector
                obj.angle = varargin{2};
            end
        end
        function omega = omega(obj)
            omega = obj.axis * obj.angle;
        end
    end
end
