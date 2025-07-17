function T=trot_z_rad(theta)
    % homogenous transformation matrix for rotation about z axis
    % theta: rad
    T = [cos(theta) -sin(theta) 0 0;
         sin(theta) cos(theta) 0 0;
         0 0 1 0;
         0 0 0 1];
end