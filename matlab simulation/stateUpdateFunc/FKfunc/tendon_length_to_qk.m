% 根据绳长计算关节角
% 输入
    % tendon_length: 绳长
% 输出
    % joint_angle: 关节角


function qk = tendon_length_to_qk(bot,state)
    k=state.k;
    num_q=size(state.qlist,2); % 关节数量
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % 目标函数
    
    f=@(qk) norm(qk(1)-state.qlist(k-1,1))+norm(qk(2)-state.qlist(k-1,2))+norm(qk(3)-state.qlist(k-1,3))+norm(qk(4)-state.qlist(k-1,4))+norm(qk(5)-state.qlist(k-1,5))+norm(qk(6)-state.qlist(k-1,6));

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
    qk=fmincon(f,state.qlist(k,:),A,b,Aeq,beq,lb,ub,nonlcon,options);

end


function [c,ceq]=set_nonlcon(qk,bot,state)
    k=state.k;
    c=[];
    [tendon_length,~]=tendon_length_update(bot,qk,state.qlist(1,:));
    ceq=tendon_length-state.tendon_length(k,:);
end