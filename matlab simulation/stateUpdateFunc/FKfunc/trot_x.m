function T = trot_x(theta)
    
    % homogeneous transformation matrix for a rotation about the x axis
    % theta: degrees
%     theta=deg2rad(theta);
    theta=theta*pi/180; % deg2rad
    T = [1 0 0 0;
         0 cos(theta) -sin(theta) 0;
         0 sin(theta) cos(theta) 0;
         0 0 0 1];
end