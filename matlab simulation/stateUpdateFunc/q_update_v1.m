% ���¹ؽڽ�
% ����
    % bot����������������˵Ĳ���
        % bot.z0 % ��ʼʱ��ǻ�������ľ��룬mm�����۳�ʼƽ���ؽ�ֵΪ���٣�z0��Ϊ��ֵ
        % bot.U_l % �������ڳ��ȣ�mm
        % bot.Link_l % link�ĳ��ȣ�����disk��ȣ���mm
        % bot.cam_l % camera_holder���ȣ�mm
        % bot.disk_h % ԲƬ��ȣ�mm
        % bot.r_hole % �׶���disk�ϵķֲ��뾶��mm
        % bot.angle_hole % �׶������rad
    % state����ǰ״̬
% ���
    % qk����һʱ�̵ĹؽڽǶ�
% ���ߣ��ž�
% ���ڣ�2023.11.16
% reference
    % [1] X. Ma, C. Song, P. W. Chiu, and Z. Li, ��Autonomous Flexible Endoscope for Minimally Invasive Surgery With Enhanced Safety,�� IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2607�C2613, Jul. 2019, doi: 10.1109/LRA.2019.2895273.
    % [2] S. Yang, Y. Wang, H. Zhao, H. Cheng, and H. Ding, "Autonomous Laparoscope Control for Minimally Invasive Surgery With Intuition and RCM Constraints,�� IEEE Robotics and Automation Letters, vol. 7, no. 3, pp. 7934�C7941, Jul. 2022, doi: 10.1109/LRA.2022.3186507.
    % [3] ���ĸ���������ѧ���������ۼ�Ӧ��ʵ�� p335-336
% ������־
    % �ž�20231120��ceq=[state.v_dot;state.u_dot]-state.J*state.Jb*q_dot_temp'; %ĩ���ٶȵ�ͼ�������ٶȵ�ӳ��
    % �ž�20231123��Jb=[state.Jb(4:6,:);state.Jb(1:3,:)]; ceq=[state.u_dot;state.v_dot]-state.J*state.Jb*q_dot_temp';
        % v���ϣ�omega����
    % �ž�20240101������������˲������ⲿ����
function qk=q_update_v1(bot,state)
    k=state.k;
    num_q=size(state.qlist,2); % �ؽ�����
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % Ŀ�꺯��
    qk_last=state.qlist(k-1,:);
    
    % f=@(qk) norm(qk(1)-state.qlist(k-1,1))+3*norm(qk(2)-state.qlist(k-1,2))+3*norm(qk(3)-state.qlist(k-1,3))+norm(qk(4)-state.qlist(k-1,4))+norm(qk(5)-state.qlist(k-1,5))+norm(qk(6)-state.qlist(k-1,6));
    f=@(qk) norm(qk(1)-qk_last(1))+3*norm(qk(2)-qk_last(2))+3*norm(qk(3)-qk_last(3))+norm(qk(4)-qk_last(4))+norm(qk(5)-qk_last(5))+norm(qk(6)-qk_last(6));

    % Լ������
    % ����ʽԼ��
    A=[];b=[];
    % ��ʽԼ��;
    Aeq=[];
    beq=[];

    dt=state.dt;
    Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
    u_dot=state.u_dot(k-1);
    v_dot=state.v_dot(k-1);
    J=state.J;
    Tse=state.Tse;
    nonlcon=@(qk)set_nonlcon(qk,bot,qk_last,dt,Jb,u_dot,v_dot,J,Tse);
    
    % �ؽڽǱ���Լ��
    lb=bot.qlim(1,:);
    ub=bot.qlim(2,:);
    % ���
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,qk_last,A,b,Aeq,beq,lb,ub,nonlcon,options);

end


% function [c,ceq]=set_nonlcon(qk,bot,state)
%     k=state.k;

%     % �ؽڽ��ٶ�Լ��
%     c=norm(qk(2:5)-state.qlist(k-1,2:5))-bot.dqlim;
   
%     ceq=[0;0;0];
%     q_dot_temp=(qk-state.qlist(k-1,:))/state.dt;
%     Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
%     ceq(1:2)=[state.u_dot(k-1);state.v_dot(k-1)]-state.J*Jb(1:3,:)*q_dot_temp'; %ĩ���ٶȵ�ͼ�������ٶȵ�ӳ��[3]
%     % ����һ����
%     % ���۾�����ϵ��
%     Qx=[1;0;0];
%     Qz=[0;0;1];
%     Vlr=[1;0;0];
%     [Tsc,~,~,~]=PUUR_Screw(bot,qk);
%     Tec=TransInv(state.Tse)*Tsc;
%     Rec=Tec(1:3,1:3);
%     ceq(3)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %������������[2]
   
% end

function [c,ceq]=set_nonlcon(qk,bot,qk_last,dt,Jb,u_dot,v_dot,J,Tse)
 
    % �ؽڽ��ٶ�Լ��
    c=zeros(1,6);
    for j=1:6
        c(j)=abs(qk(j)-qk_last(j))-bot.dqlim(j);
    end
   
    ceq=[0;0;0];
    q_dot_temp=(qk-qk_last)/dt;
    ceq(1:2)=[u_dot;v_dot]-J*Jb(1:3,:)*q_dot_temp'; %ĩ���ٶȵ�ͼ�������ٶȵ�ӳ��[3]
    % ����һ����
    % ���۾�����ϵ��
    Qx=[1;0;0];
    Qz=[0;0;1];
    Vlr=[1;0;0];
    [Tsc,~,~,~]=PUUR_Screw(bot,qk);
    Tec=TransInv(Tse)*Tsc;
    Rec=Tec(1:3,1:3);
    ceq(3)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %������������[2]
   
end