% 更新关节角
% 输入
    % c:模式
    % bot：描述连续体机器人的参数
        % bot.z0 % 初始时腹腔镜与底面的距离，mm，无论初始平动关节值为多少，z0均为该值
        % bot.U_l % 半个万向节长度，mm
        % bot.Link_l % link的长度（不计disk厚度），mm
        % bot.cam_l % camera_holder长度，mm
        % bot.disk_h % 圆片厚度，mm
        % bot.r_hole % 孔洞在disk上的分布半径，mm
        % bot.angle_hole % 孔洞间隔，rad
    % state：当前状态
    % err_des：允许中心误差
    % r_ded：期望半径
    % r_thr：允许半径误差
% 输出
    % qk：下一时刻的关节角度
% 作者：张晶
% 日期：2023.12.29
% 更新日志
    % 张晶20240101：连续体机器人参数由外部输入

function qk=q_update(adjust_r,bot,state,err_des,r_thr,track_always)
    k=state.k;
    if track_always % 总是跟随模式
        if norm(state.pix_err(:,k-1))==0
            if adjust_r
                qk=q_update_for_r(bot,state);
            else
                qk=state.qlist(k-1,:);
            end
        elseif adjust_r
            qk=q_update_v2(bot,state); % 由优化函数得到，1st ver done in 20231113
        else
            qk=q_update_v1(bot,state); % 2nd ver done in 20231129
        end
    else % 达到阈值就不跟随
        if norm(state.pix_err(:,k-1)) < err_des
            if ~adjust_r || state.r_err(k-1) < r_thr % 中心和半径均满足约束
                qk=state.qlist(k-1,:);
            else 
                qk=q_update_for_r(bot,state);
            end
        elseif ~adjust_r
            qk=q_update_v1(bot,state); % 由优化函数得到，1st ver done in 20231113
        else
            qk=q_update_v2(bot,state); % 2nd ver done in 20231129
        end
    end
end