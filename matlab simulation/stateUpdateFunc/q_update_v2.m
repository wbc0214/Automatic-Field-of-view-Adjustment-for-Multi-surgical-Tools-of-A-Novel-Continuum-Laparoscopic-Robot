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
    % �ž�2023.11.29��copy from q_update�����Ӱ뾶��Լ��
    % �ž�20240101������������˲������ⲿ����
function qk=q_update_v2(bot,state)
    k=state.k;
    num_q=size(state.qlist,2); % �ؽ�����
    % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
    % Ŀ�꺯��
    f=@(qk) norm(qk(1)-state.qlist(k-1,1))+3*norm(qk(2)-state.qlist(k-1,2))+3*norm(qk(3)-state.qlist(k-1,3))+norm(qk(4)-state.qlist(k-1,4))+norm(qk(5)-state.qlist(k-1,5))+norm(qk(6)-state.qlist(k-1,6));
    
    % Լ������
    % ����ʽԼ��
    A=[];b=[];
    % ��ʽԼ��;
    Aeq=[];
    beq=[];
    nonlcon=@(qk)set_nonlcon(qk,bot,state);
    
    % �ؽڽǱ���Լ��
    lb=bot.qlim(1,:);
    ub=bot.qlim(2,:);

    % ���
    options = optimoptions('fmincon','Algorithm','sqp','Display','off');
    qk=fmincon(f,state.qlist(k-1,:),A,b,Aeq,beq,lb,ub,nonlcon,options);

end


function [c,ceq]=set_nonlcon(qk,bot,state)
    k=state.k;
    % �ؽڽ��ٶ�Լ��
    c=zeros(1,6);
    for j=1:6
        c(j)=abs(qk(j)-state.qlist(k-1,j))-bot.dqlim(j);
    end

    q_dot_temp=(qk-state.qlist(k-1,:))/state.dt;
    Jb=[state.Jb(4:6,:);state.Jb(1:3,:)];
    V=Jb*q_dot_temp'; % [v,w]

    %ĩ���ٶȵ�ͼ�������ٶȵ�ӳ��[3]
    ceq=[state.u_dot(k-1);state.v_dot(k-1)]-state.J*V(1:3); 


    % ����һ����
    % ���۾�����ϵ��
    Qx=[1;0;0];
    Qz=[0;0;1];
    Vlr=[1;0;0];
    [Tsc,~,~,~]=PUUR_Screw(bot,qk);
    Tec=TransInv(state.Tse)*Tsc;
    Rec=Tec(1:3,1:3);
    ceq(3)=(Rec*Qx)'*cross(Rec*Qz,Vlr); %������������[2]


%     xc=[1;0;0];
%     zc=[0;0;1];
%     xe=[1;0;0];
%     [Tsc,~,~,~]=PUUR_Screw(bot,qk);
%     Tec=TransInv(state.Tse)*Tsc;
%     Rec=Tec(1:3,1:3);
%     % ���۾�����ϵ��
%     n=cross(xe,Rec*zc); % ƽ�淨����
%     xc_ine=Rec*xc;
%     n_cross_xc_ine=cross(n,xc_ine);
%     % ����n��xc_ine�ļнǣ���ΧΪ-pi��pi
%     if n_cross_xc_ine(3)>0
%         theta=atan2d(-norm(cross(n,xc_ine)),dot(n,xc_ine));% deg
%     else
%         theta=atan2d(norm(cross(n,xc_ine)),dot(n,xc_ine));% deg
%     end
% 
%     % ��ƽ����xc_ine�ļн�
%     if theta>-180 && theta<=90
%         eye_hand_err=theta+90;% deg
%     else
%         eye_hand_err=theta-270;% deg
%     end
%     ceq(3)=eye_hand_err;
   
    % Ŀ��Բ�뾶Լ��
    
    Kp=3;
    Kd=0.1;
    if k > 3
        ceq(4)=Kp*state.r_err(k-1)+Kd*(state.r_err(k-1)-state.r_err(k-2))-V(3);
%     else
%         ceq(4)=Kp*state.r_err(k-1)-V(3);
    end
end