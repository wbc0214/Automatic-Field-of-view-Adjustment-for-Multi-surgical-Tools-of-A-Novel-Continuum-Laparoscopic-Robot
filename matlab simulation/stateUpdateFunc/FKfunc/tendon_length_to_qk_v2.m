% ������������ؽڽ�
% ����
    % tendon_length: ����
% ���
    % joint_angle: �ؽڽ�


function qk = tendon_length_to_qk_v2(bot,qk_last,qk_init,desired_tendon_length)
    num_q=6; % �ؽ�����
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % Ŀ�꺯��
    
    f=@(qk) norm(qk(1)-qk_last(1))+3*norm(qk(2)-qk_last(2))+3*norm(qk(3)-qk_last(3))+norm(qk(4)-qk_last(4))+norm(qk(5)-qk_last(5))+norm(qk(6)-qk_last(6));
    % f=@(qk) norm(qk(6));
    % Լ������
    % ����ʽԼ��
    A=[];b=[];
    % ��ʽԼ��;
    Aeq=[];
    beq=[];
    nonlcon=@(qk)set_nonlcon(qk,bot,desired_tendon_length,qk_init,qk_last);
    
    % �ؽڽǱ���Լ��
    lb=ones(1,num_q)*bot.qlim(1);
    ub=ones(1,num_q)*bot.qlim(2);
    lb(1)=bot.q1lim(1);
    ub(1)=bot.q1lim(2);
    lb(num_q)=-pi;
    ub(num_q)=pi;

    % ���
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,qk_init,A,b,Aeq,beq,lb,ub,nonlcon,options);

end


function [c,ceq]=set_nonlcon(qk,bot,desired_tendon_length,qk_init,qk_last)
    % �ؽڽ��ٶ�Լ��
    % c=zeros(1,6);
    % for j=1:6
    %     c(j)=norm(qk(j)-qk_last(j))-bot.dqlim;
    % end
    c=[]; 
    [tendon_length,~]=tendon_length_update(bot,qk,qk_init);
    ceq=tendon_length-desired_tendon_length;
end