function T = transl_z(z)
    % transl_z  homogeneous transformation for translation along z
    T=[
        1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, z;
        0, 0, 0, 1;
    ];
end