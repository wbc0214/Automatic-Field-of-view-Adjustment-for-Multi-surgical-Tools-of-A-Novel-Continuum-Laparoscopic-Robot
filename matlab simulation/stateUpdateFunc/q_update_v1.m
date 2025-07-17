% 更新关节角
% 输入
    % bot：描述连续体机器人的参数
        % bot.z0 % 初始时腹腔镜与底面的距离，mm，无论初始平动关节值为多少，z0均为该值
        % bot.U_l % 半个万向节长度，mm
        % bot.Link_l % link的长度（不计disk厚度），mm
        % bot.cam_l % camera_holder长度，mm
        % bot.disk_h % 圆片厚度，mm
        % bot.r_hole % 孔洞在disk上的分布半径，mm
        % bot.angle_hole % 孔洞间隔，rad
    % state：当前状态
% 输出
    % qk：下一时刻的关节角度
% 作者：张晶
% 日期：2023.11.16
% reference
    % [1] X. Ma, C. Song, P. W. Chiu, and Z. Li, “Autonomous Flexible Endoscope for Minimally Invasive Surgery With Enhanced Safety,” IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2607–2613, Jul. 2019, doi: 10.1109/LRA.2019.2895273.
    % [2] S. Yang, Y. Wang, H. Zhao, H. Cheng, and H. Ding, "Autonomous Laparoscope Control for Minimally Invasive Surgery With Intuition and RCM Constraints,” IEEE Robotics and Automation Letters, vol. 7, no. 3, pp. 7934–7941, Jul. 2022, doi: 10.1109/LRA.2022.3186507.
    % [3] 徐文福，机器人学：基础理论及应用实践 p335-336
% 更新日志
    % 张晶20231120：ceq=[state.v_dot;state.u_dot]-state.J*state.Jb*q_dot_temp'; %末端速度到图像特征速度的映射
    % 张晶20231123：Jb=[state.Jb(4:6,:);state.Jb(1:3,:)]; ceq=[state.u_dot;state.v_dot]-state.J*state.Jb*q_dot_temp';
        % v在上，omega在下
    % 张晶20240101：连续体机器人参数由外部输入
function qk=q_update_v1(bot,state)
    k=state.k;
    num_q=size(state.qlist,2); % 关节数量
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % 目标函数
    qk_last=state.qlist(k-1,:);
    
    % f=@(qk) norm(qk(1)-state.qlist(k-1,1))+3*norm(qk(2)-state.qlist(k-1,2))+3*norm(qk(3)-state.qlist(k-1,3))+norm(qk(4)-state.qlist(k-1,4))+norm(qk(5)-state.qlist(k-1,5))+norm(qk(6)-state.qlist(k-1,6));
    f=@(qk) norm(qk(1)-qk_last(1))+3*norm(qk(2)-qk_last(2))+3*norm(qk(3)-qk_last(3))+norm(qk(4)-qk_last(4))+norm(qk(5)-qk_last(5))+norm(qk(6)-qk_last(6));

    % 约束条件
    % 不等式约束
    A=[];b=[];
    % 等式约束;
    Aeq=[];
    beq=[];

    dt=state.dt;
    Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
    u_dot=state.u_dot(k-1);
    v_dot=state.v_dot(k-1);
    J=state.J;
    Tse=state.Tse;
    nonlcon=@(qk)set_nonlcon(qk,bot,qk_last,dt,Jb,u_dot,v_dot,J,Tse);
    
    % 关节角变量约束
    lb=bot.qlim(1,:);
    ub=bot.qlim(2,:);
    % 求解
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,qk_last,A,b,Aeq,beq,lb,ub,nonlcon,options);

end


% function [c,ceq]=set_nonlcon(qk,bot,state)
%     k=state.k;

%     % 关节角速度约束
%     c=norm(qk(2:5)-state.qlist(k-1,2:5))-bot.dqlim;
   
%     ceq=[0;0;0];
%     q_dot_temp=(qk-state.qlist(k-1,:))/state.dt;
%     Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
%     ceq(1:2)=[state.u_dot(k-1);state.v_dot(k-1)]-state.J*Jb(1:3,:)*q_dot_temp'; %末端速度到图像特征速度的映射[3]
%     % 手眼一致性
%     % 在眼睛坐标系下
%     Qx=[1;0;0];
%     Qz=[0;0;1];
%     Vlr=[1;0;0];
%     [Tsc,~,~,~]=PUUR_Screw(bot,qk);
%     Tec=TransInv(state.Tse)*Tsc;
%     Rec=Tec(1:3,1:3);
%     ceq(3)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %三个向量共面[2]
   
% end

function [c,ceq]=set_nonlcon(qk,bot,qk_last,dt,Jb,u_dot,v_dot,J,Tse)
 
    % 关节角速度约束
    c=zeros(1,6);
    for j=1:6
        c(j)=abs(qk(j)-qk_last(j))-bot.dqlim(j);
    end
   
    ceq=[0;0;0];
    q_dot_temp=(qk-qk_last)/dt;
    ceq(1:2)=[u_dot;v_dot]-J*Jb(1:3,:)*q_dot_temp'; %末端速度到图像特征速度的映射[3]
    % 手眼一致性
    % 在眼睛坐标系下
    Qx=[1;0;0];
    Qz=[0;0;1];
    Vlr=[1;0;0];
    [Tsc,~,~,~]=PUUR_Screw(bot,qk);
    Tec=TransInv(Tse)*Tsc;
    Rec=Tec(1:3,1:3);
    ceq(3)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %三个向量共面[2]
   
end