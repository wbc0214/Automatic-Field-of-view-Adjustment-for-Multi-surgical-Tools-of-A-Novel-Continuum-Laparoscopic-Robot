
L=100;% length of Link




DH_Table=[% theta d   a   alpha
              0   0   0    180;% 旋转关节，但不转
              0   0   0     90;%平动关节
             90   0   0     90;
              0   0   L      0;
            180   0   0     90;
            -90  0   0      90;
             90   L   0      0;             
];
% q_deg=[0 0 0 0 0 0 0]; % 关节角度

T0=eye(4);
T1=T0*trot_z(DH_Table(1,1))*transl_z(DH_Table(1,2))*transl_x(DH_Table(1,3))*trot_x(DH_Table(1,4));
T2=T1*trot_z(DH_Table(2,1))*transl_z(q_deg(2)+DH_Table(2,2))*transl_x(DH_Table(2,3))*trot_x(DH_Table(2,4));
T3=T2*trot_z(q_deg(3)+DH_Table(3,1))*transl_z(DH_Table(3,2))*transl_x(DH_Table(3,3))*trot_x(DH_Table(3,4));
T4=T3*trot_z(q_deg(4)+DH_Table(4,1))*transl_z(DH_Table(4,2))*transl_x(DH_Table(4,3))*trot_x(DH_Table(4,4));
T5=T4*trot_z(q_deg(5)+DH_Table(5,1))*transl_z(DH_Table(5,2))*transl_x(DH_Table(5,3))*trot_x(DH_Table(5,4));
T6=T5*trot_z(q_deg(6)+DH_Table(6,1))*transl_z(DH_Table(6,2))*transl_x(DH_Table(6,3))*trot_x(DH_Table(6,4));
T_end=T6*trot_z(q_deg(7)+DH_Table(7,1))*transl_z(DH_Table(7,2))*transl_x(DH_Table(7,3))*trot_x(DH_Table(7,4));

% abs(T_end-T_end_bot)<1e-4