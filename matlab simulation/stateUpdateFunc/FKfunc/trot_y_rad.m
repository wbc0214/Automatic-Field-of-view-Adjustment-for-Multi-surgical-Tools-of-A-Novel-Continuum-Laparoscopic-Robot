function T=trot_y_rad(theta)
    % homogenous transformation matrix for rotation about y axis
    % theta: rad
    T=[cos(theta) 0 sin(theta) 0;
        0 1 0 0;
        -sin(theta) 0 cos(theta) 0;
        0 0 0 1];
end