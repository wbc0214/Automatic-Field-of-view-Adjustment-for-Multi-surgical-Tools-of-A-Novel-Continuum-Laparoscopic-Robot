% 仅半径需要调整时，更新关节角
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
% 日期：2023.12.29
% 更新日志
    % 张晶20240101：连续体机器人参数由外部输入


function qk=q_update_for_r(bot,state)
    k=state.k;
    num_q=size(state.qlist,2); % 关节数量
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % 目标函数
    f=@(qk) norm(qk(1)-state.qlist(k-1,1))+2*norm(qk(2)-state.qlist(k-1,2))+2*norm(qk(3)-state.qlist(k-1,3))+norm(qk(4)-state.qlist(k-1,4))+norm(qk(5)-state.qlist(k-1,5))+norm(qk(6)-state.qlist(k-1,6));
    
    % 约束条件
    % 不等式约束
    A=[];b=[];
    % 等式约束;
    Aeq=[];
    beq=[];
    nonlcon=@(qk)set_nonlcon(qk,bot,state);
    
    % 关节角变量约束
    lb=ones(1,num_q)*state.qlim(1);
    ub=ones(1,num_q)*state.qlim(2);
    lb(1)=state.q1lim(1);
    ub(1)=state.q1lim(2);
    lb(num_q)=-pi;
    ub(num_q)=pi;

    % 求解
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,state.qlist(k-1,:),A,b,Aeq,beq,lb,ub,nonlcon,options);

end


function [c,ceq]=set_nonlcon(qk,bot,state)
    k=state.k;
    c=[];
    q_dot_temp=(qk-state.qlist(k-1,:))/state.dt;
    Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
    V=Jb*q_dot_temp'; % [v,w]
    % 目标圆半径约束
    Kp=3;
    Kd=0.1;
    if k>2
        V_des=[0;0;Kp*state.r_err(k-1)+Kd*(state.r_err(k-1)-state.r_err(k-2))];
    else
        V_des=[0;0;Kp*state.r_err(k-1)];
    end
    %末端速度到图像特征速度的映射[3]
    ceq=V_des-V(1:3); 


    % 手眼一致性
    % 在眼睛坐标系下
    Qx=[1;0;0];
    Qz=[0;0;1];
    Vlr=[1;0;0];
    [Tsc,~,~,~]=PUUR_Screw(bot,qk);
    Tec=TransInv(state.Tse)*Tsc;
    Rec=Tec(1:3,1:3);
    ceq(4)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %三个向量共面[2]
   
end