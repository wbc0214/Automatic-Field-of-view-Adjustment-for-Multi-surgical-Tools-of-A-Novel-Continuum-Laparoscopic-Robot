function [Tsb,Tbs,Js,Jb]=PUUR_Screw_webots(thetalist)
    L=0.5;
    z_base=4;
    n=2;
    z0=z_base-(n+1)*L;

    M=[
        0,  1,  0,  0;
        1,  0,  0,  0;
        0,  0, -1,  z0;
        0,  0,  0,  1
       ];
    Slist=[
            0, 0, 0,     0,     0,   -1;
            0, 1, 0, -2*L-z0,   0,    0;
            1, 0, 0,     0,  2*L+z0,  0;
            1, 0, 0,     0,    L+z0,    0;
            0, 1, 0,   -L-z0,   0,    0;
            0, 0, -1,    0,     0,    0;
          ]';

    Tsb = FKinSpace(M, Slist, thetalist');
    Js  = JacobianSpace(Slist, thetalist'); % 关节角--> 末端空间速度
    Tbs = TransInv(Tsb);
    Jb  = Adjoint(Tbs)*Js;
end