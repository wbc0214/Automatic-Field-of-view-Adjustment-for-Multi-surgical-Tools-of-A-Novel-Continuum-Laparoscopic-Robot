% 求解给定关节角下的末端齐次变换阵和雅可比矩阵
% PUUR：1个沿z轴的平动关节+2个xy向的万向节+1个绕z轴的旋转关节
% 输入
    % bot：描述连续体机器人的参数
    % thetalist：关节转动（平移）量，rad（mm）
        % 平动关节的平移量指的是RCM约束处到第一个万向节的中心的距离
% 输出
    % Tsb：末端坐标系相对于世界坐标系的齐次变换阵
    % Tbs：
    % Js：空间雅可比
    % Jb：物体雅可比
% 作者：张晶
% 日期：2023.11.16
% 修改日志
    % 张晶20231120：增加Tbs的输出
    % 张晶20240101：连续体机器人参数由外部输入
    % 张晶20240101：写出Slist具体求解过程，便于检查

function [Tsb,Tbs,Js,Jb]=PUUR_Screw(bot,thetalist)
    
    L=bot.U_l*2+bot.disk_h*2+bot.Link_l;% 两个万向节中心的距离
    cam_L=bot.U_l+bot.cam_l; % 第二个万向节中心到连续体机器人末端的距离
    z0=bot.z0; % 相机初始位置，mm

    % 腹腔镜初始位姿
    M=[
        1,  0,  0,  0;
        0,  -1, 0,  0;
        0,  0, -1,  z0
        0,  0,  0,  1
       ];
    % 两个万向节
    w1=[0;0;0];v1=[0;0;-1];
    w2=[0;1;0];q2=[0;0;z0+cam_L+L];v2=-cross(w2,q2);
    w3=[1;0;0];q3=q2;v3=-cross(w3,q3);
    w4=w3;q4=[0;0;z0+cam_L];v4=-cross(w4,q4);
    w5=w2;q5=q4;v5=-cross(w5,q5);
    w6=[0;0;-1];q6=[0;0;0];v6=-cross(w6,q6);
    Slist=[
            w1,w2,w3,w4,w5,w6;
            v1,v2,v3,v4,v5,v6;
          ];
   
    Tsb = FKinSpace(M, Slist, thetalist');
    Js  = JacobianSpace(Slist, thetalist');
    Tbs = TransInv(Tsb);
    Jb  = Adjoint(Tbs)*Js;

%     Tsb = FKinBody(M, Blist, thetalist');
%     Jb = JacobianBody(Blist, thetalist');
%     Adjoint(Tsb)*Jb;
 
end