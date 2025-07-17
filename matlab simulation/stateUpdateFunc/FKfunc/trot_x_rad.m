function T = trot_x_rad(theta)
    
    % homogeneous transformation matrix for a rotation about the x axis
    % theta: rad
    T = [1 0 0 0;
         0 cos(theta) -sin(theta) 0;
         0 sin(theta) cos(theta) 0;
         0 0 0 1];
end