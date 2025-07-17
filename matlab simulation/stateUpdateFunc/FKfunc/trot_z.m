function T=trot_z(theta)
    % homogenous transformation matrix for rotation about z axis
    % theta: degrees
%     theta=deg2rad(theta);
    theta=theta*pi/180; % deg2rad
    T = [cos(theta) -sin(theta) 0 0;
         sin(theta) cos(theta) 0 0;
         0 0 1 0;
         0 0 0 1];
end