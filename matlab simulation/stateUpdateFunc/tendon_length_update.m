% 计算体内部分绳长
% 作者：张晶
% 日期：2023.12.30

% 输入
    % bot：描述连续体机器人的参数
        % bot.z0 % 初始时腹腔镜与底面的距离，mm，无论初始平动关节值为多少，z0均为该值
        % bot.U_l % 半个万向节长度，mm
        % bot.Link_l % link的长度（不计disk厚度），mm
        % bot.cam_l % camera_holder长度，mm
        % bot.disk_h % 圆片厚度，mm
        % bot.r_hole % 孔洞在disk上的分布半径，mm
        % bot.angle_hole % 孔洞间隔，rad
    % qlist：关节转动（平移）量，rad
        % 平动关节的平移量指的是RCM约束处到第一个万向节的中心的距离
% 输出
    % tendon_length：各个绳长

% PUUR
% q1_real=0时，P与disk连接处在RCM处
% qlist(1)=0时，U1的中心点在RCM处
% 在RCM处建立{0}坐标系（非世界坐标系）
% {1}：P与U1的disk连接处
% {2}：U1与Link连接处
% {3}：Link与U2连接处
% {4}：U2与R连接处
% x轴正方向为0号孔

% U1：4、10、16
% U2：5、11、17
% R：0、9

% 更新日志
    % 张晶2024.06.20：机器人改为驱动部分在平动关节上
    

function [tendon_length,Tlist]=tendon_length_update(bot,qlist,qlist_init)
    L=bot.U_l*2+bot.disk_h*2+bot.Link_l;% 两个万向节中心的距离
    cam_L=bot.U_l+bot.cam_l; % 第二个万向节中心到连续体机器人末端的距离
    z0=bot.z0; % 相机初始位置，mm
    T0=[ 1,  0,  0,  0;
         0, -1,  0,  0;
         0,  0, -1,  z0+cam_L+L;
         0,  0,  0,   1;
    ];
    % q1_real=0时，P与disk连接处在RCM处
    % qlist(1)=0时，U1的中心点在RCM处
    q1_real=qlist(1)-bot.U_l;
    T1=T0*transl_z(q1_real);
    T2=T1*transl_z(bot.U_l)*trot_y_rad(-qlist(2))*trot_x_rad(qlist(3))*transl_z(bot.U_l+bot.disk_h);
    T3=T2*transl_z(bot.Link_l+bot.disk_h);
    T4=T3*transl_z(bot.U_l)*trot_x_rad(qlist(4))*trot_y_rad(-qlist(5))*transl_z(bot.U_l+bot.disk_h);
    Tc=T4*trot_z_rad(qlist(6))*transl_z(bot.cam_l-bot.disk_h);
    abs(Tc-PUUR_Screw(bot,qlist))<1e-10; % 正向运动学正确检验
    
    % 当前绳长
    % U1
%     w2=[0;-1;0];q2=[0;0;bot.U_l];v2=-cross(w2,q2);
%     S2=[w2;v2];
%     w3=[1;0;0];q3=q2;v3=-cross(w3,q3);
%     S3=[w3;v3];
%     l_v2=get_length_v2(bot,S2,S3,qlist(2),qlist(3),4)
%     l=get_length(T1,T2,bot,4)
    
    L4=get_length(T1,T2,bot,4);
    L10=get_length(T1,T2,bot,10);
    L16=get_length(T1,T2,bot,16);
    % U2
    L5=get_length(T1,T2,bot,5)+bot.Link_l+bot.disk_h+get_length(T3,T4,bot,5);
    L11=get_length(T1,T2,bot,11)+bot.Link_l+bot.disk_h+get_length(T3,T4,bot,11);
    L17=get_length(T1,T2,bot,17)+bot.Link_l+bot.disk_h+get_length(T3,T4,bot,17);
    % R
    Lq6=0; % TODO
    delta_Lq6=(qlist(6)-qlist_init(6))*sqrt(bot.rh^2+bot.ph^2); 
    % R正转时0号线变短，9号线变长
    L0=get_length(T1,T2,bot,0)+bot.Link_l+bot.disk_h+get_length(T3,T4,bot,0)+Lq6+delta_Lq6;
    L9=get_length(T1,T2,bot,9)+bot.Link_l+bot.disk_h+get_length(T3,T4,bot,9)+Lq6-delta_Lq6;
    
    tendon_length=[L4 L10 L16 L5 L11 L17 L0 L9];
    Tlist=[T0;T1;T2;T3;T4;Tc];

end