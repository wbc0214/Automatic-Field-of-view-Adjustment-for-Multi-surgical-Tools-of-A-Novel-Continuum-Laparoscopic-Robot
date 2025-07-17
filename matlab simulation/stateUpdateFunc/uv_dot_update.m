% 更新目标点在图像上期望的移动速度（含大小和方向）
% 作者：张晶
% 日期：2023.11.24
% 输入
    % state：当前状态
    % [u_des, v_des]：目标点期望位置的像素坐标
% 输出
    % [u_dot, v_dot]：目标点在图像上期望的移动速度
% 更新日志
    % 张晶20231205：PID控制器调节速度大小
function [u_dot, v_dot]=uv_dot_update(state,u_des,v_des,PID_param)
    Kp=PID_param.Kp;
    Kd=PID_param.Kd; 
    Ki=PID_param.Ki;
    decay=PID_param.decay;
    k=state.k; % 当前迭代步
    
    dir=[u_des-state.ulist(k);v_des-state.vlist(k)]/norm([u_des-state.ulist(k);v_des-state.vlist(k)]); % pix error
    
    sum_err=[0;0];
    for i=1:k
        sum_err=decay*sum_err+state.pix_err(:,i);
    end
    if k==1
        lambda=Kp*state.pix_err(:,k);
%     elseif norm(state.pix_err(:,k)) < 100
    else
        d_err=state.pix_err(:,k)-state.pix_err(:,k-1);
        lambda=Kp*state.pix_err(:,k)+Kd*d_err+Ki*sum_err;
%     else
%         d_err=state.pix_err(:,k)-state.pix_err(:,k-1);
%         lambda=Kp*state.pix_err(:,k)+Kd*d_err;
    end
    uv_dot=abs(lambda).*dir;
    u_dot=uv_dot(1);
    v_dot=uv_dot(2);
end