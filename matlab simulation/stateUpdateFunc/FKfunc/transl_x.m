function T = transl_x(x)
    % transl_z  homogeneous transformation for translation along z
    T=[
        1, 0, 0, x;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1;
    ];
end