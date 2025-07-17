% 根据绳长计算关节角
% 输入
    % tendon_length: 绳长
% 输出
    % joint_angle: 关节角


function qk = tendon_length_to_qk_v2(bot,qk_last,qk_init,desired_tendon_length)
    num_q=6; % 关节数量
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % 目标函数
    
    f=@(qk) norm(qk(1)-qk_last(1))+3*norm(qk(2)-qk_last(2))+3*norm(qk(3)-qk_last(3))+norm(qk(4)-qk_last(4))+norm(qk(5)-qk_last(5))+norm(qk(6)-qk_last(6));
    % f=@(qk) norm(qk(6));
    % 约束条件
    % 不等式约束
    A=[];b=[];
    % 等式约束;
    Aeq=[];
    beq=[];
    nonlcon=@(qk)set_nonlcon(qk,bot,desired_tendon_length,qk_init,qk_last);
    
    % 关节角变量约束
    lb=ones(1,num_q)*bot.qlim(1);
    ub=ones(1,num_q)*bot.qlim(2);
    lb(1)=bot.q1lim(1);
    ub(1)=bot.q1lim(2);
    lb(num_q)=-pi;
    ub(num_q)=pi;

    % 求解
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,qk_init,A,b,Aeq,beq,lb,ub,nonlcon,options);

end


function [c,ceq]=set_nonlcon(qk,bot,desired_tendon_length,qk_init,qk_last)
    % 关节角速度约束
    % c=zeros(1,6);
    % for j=1:6
    %     c(j)=norm(qk(j)-qk_last(j))-bot.dqlim;
    % end
    c=[]; 
    [tendon_length,~]=tendon_length_update(bot,qk,qk_init);
    ceq=tendon_length-desired_tendon_length;
end