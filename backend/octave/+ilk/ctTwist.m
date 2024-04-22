function ctTwist(newframe_H_current, twist_current, twist_newframe)
    % We need to perform a spatial motion vector coordinate transform, given
    % the homogeneous coordinate transform for the same frames.
    % Check e.g. chapter 2 of Roy's RBDA, for the relation between the two
    % transforms, to understand the last line of this function

    coords_ang = ilk.angularCoords();
    coords_lin = ilk.linearCoords();

    R = newframe_H_current(1:3,1:3);             % 3x3 rotation matrix
    w = twist_current(coords_ang);          % omega in the current coordinates
    twist_newframe(coords_ang) = R * w;  % omega in the new coordinates
    twist_newframe(coords_lin) = R * twist_current(coords_lin) +...
                           cross( newframe_H_current(1:3,4),...
                                  twist_newframe(coords_ang)  );
end
