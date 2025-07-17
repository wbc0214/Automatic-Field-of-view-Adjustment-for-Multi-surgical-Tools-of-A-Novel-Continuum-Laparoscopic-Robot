function T=trot_y(theta)
    % homogenous transformation matrix for rotation about y axis
    % theta: degrees
%     theta=deg2rad(theta);
    theta=theta*pi/180; % deg2rad
    T=[cos(theta) 0 sin(theta) 0;
        0 1 0 0;
        -sin(theta) 0 cos(theta) 0;
        0 0 0 1];
end