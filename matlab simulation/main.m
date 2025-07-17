% 作者：张晶
% 更新日志
    % 张晶2023.12.08：增加case 4 one moving circle
    % Zc没有用
    % 张晶2024.04.01：将init步骤放到主循环中，更新运行逻辑
% 
clear;
clc
close all;
addpath(genpath("drawResult\"),genpath("stateUpdateFunc\"));
% 速度用差分形式来刻画
% 用state封装连续体机器人关节角等信息
% 用camera封装相机参数
% 用bot封装连续体机器人参数
run("bot_args.m"); % 机器人参数
run("state_init.m"); % 初始化状态
state.qlist(1,:)=[20 0 0 0 0 0.3];% 初始关节角，rad 平动关节，mm
%% 设置相机参数
camera.imageSize = [640, 480]; % 图像尺寸
camera.fu=500;
camera.fv=500;
camera.u0=320;
camera.v0=240;
% 创建相机内参对象
camera.focalLength = [camera.fu, camera.fv]; % 焦距
camera.principalPoint = [camera.u0, camera.v0]; % 主点坐标
camera.intrinsics = cameraIntrinsics(camera.focalLength, camera.principalPoint, camera.imageSize); % 相机内参
% 目标位置
u_des=camera.imageSize(1)/2;
v_des=camera.imageSize(2)/2;

%% 设置仿真参数
adjust_r=true; %是否调整半径
draw_camera_view=false; % 在迭代过程中是否显示相机视角
track_always=true;

%     traj_shape_list=[ 
%         "point",... % 单个固定点
%         "points",... % 多个固定点
%         "line2D",... % 2D直线
%         "line3D",... % 3D直线
%         "circle2D",... % 2D圆
%         "circle3D" % [1] eq(31)
%     ];

traj_shape="cut"; % 轨迹形状
T=3; % 仿真时间
r_des=100; % 期望半径 
r_thr=10; % 半径阈值
err_des=30; % 允许误差 px

% PID参数，决定了 Zc[u_dot;v_dot] 的大小
PID_param.Kp=80*eye(2);
PID_param.Ki=50*eye(2);
PID_param.Kd=20*eye(2); 
PID_param.decay=0.8;

%% 主循环
tic
start_time=datetime("now");
while true
    
    current_time=datetime("now");
    t=seconds(current_time-start_time);
    if t > T % 仿真时间
        state.k=state.k-1;
        break
    end
    step_start_time=datetime("now"); % 统计单次迭代时间，开始时间

    % 更新相机的位姿及雅可比矩阵
    [state.Tsc,state.Tcs,state.Js,state.Jb]=PUUR_Screw(bot,state.qlist(k,:));
    % 更新相机投影矩阵
    camera.tform = rigidtform3d(state.Tcs(1:3,1:3), state.Tcs(1:3,4));
    camera.camProjection = cameraProjection(camera.intrinsics, camera.tform);
    % 更新状态，并记录当前误差
    [target,target_bound,~,target_num] = get_target(traj_shape,T,t);
    if target_num == 1
        target_list(k,:)=target;target_bound_list(k,:)=target_bound;
        [state.ulist(k),state.vlist(k),state.Zclist(k),state.r(k)]=uv_r_update(camera,state,target,target_bound,draw_camera_view);
    else
        target_list1(k,:)=target(1,:);target_bound_list1(k,:)=target_bound(1,:);
        target_list2(k,:)=target(2,:);target_bound_list2(k,:)=target_bound(2,:);
        target_list(k,:)=(target_list1(k,:)+target_list2(k,:))/2;target_bound_list(k,:)=(target_bound_list1(k,:)+target_bound_list2(k,:))/2;
        [state.ulist1(k),state.vlist1(k),state.Zclist1(k),state.r1(k)]=uv_r_update(camera,state,target(1,:),target_bound(1,:),draw_camera_view);
        [state.ulist2(k),state.vlist2(k),state.Zclist2(k),state.r2(k)]=uv_r_update(camera,state,target(2,:),target_bound(2,:),draw_camera_view);
        state.ulist(k)=(state.ulist1(k)+state.ulist2(k))/2;state.vlist(k)=(state.vlist1(k)+state.vlist2(k))/2;state.r(k)=(state.r1(k)+state.r2(k))/2;
        state.Zclist(k)=(state.Zclist1(k)+state.Zclist2(k))/2;
    end
    state.pix_err(:,k)=[u_des-state.ulist(k);v_des-state.vlist(k)];
    state.r_err(k)=r_des-state.r(k);

    % 更新末端速度到图像特征速度的映射J
    state.J=J_update(camera,state,state.Zclist(k));
    % 更新速度，实际为Zc[u_dot;v_dot]
    [state.u_dot(k), state.v_dot(k)]=uv_dot_update(state,u_des,v_des,PID_param);

    % 更新状态序号k
    state.k=state.k+1;k=state.k;
    
    % 更新关节角及绳度
    state.qlist(k,:)=q_update(adjust_r,bot,state,err_des,r_thr,track_always);
    [state.tendon_length(1,:),~]=tendon_length_update(bot,state.qlist(1,:),state.qlist(1,:));
    [state.tendon_length(k,:),~]=tendon_length_update(bot,state.qlist(k,:),state.qlist(1,:));
    tendon_length=state.tendon_length(k,:)-state.tendon_length(1,:);

    % 统计单次迭代时间
    step_end_time=datetime("now");
    state.duration(k-1)=seconds(step_end_time-step_start_time);
end
toc

%% 保存数据并绘图
% 保存数据
current_time=datetime("now");
result_path=['result/sim/',char(traj_shape),'/',num2str(current_time.Year),'_',num2str(current_time.Month,'%02u'),'_',num2str(current_time.Day,'%02u'),'_',num2str(current_time.Hour,'%02u'),'_',num2str(current_time.Minute,'%02u'),'_',num2str(floor(current_time.Second),'%02u')];
% 创建结果文件夹
mkdir(result_path);
% 保存参数
save([result_path,'/result.mat']);


% 绘制结果
run("drawTraj.m")
run("drawTrajWithoutTracking.m")
run("drawConvergenceSteps.m")   % 追踪误差收敛过程
%if adjust_r
    %run("drawRSteps.m")  % 绘制半径变化
%end
%run("drawDuration.m")   % 绘制每部计算时长
%run("drawErrV.m")   % 绘制误差与相机速度
%run("drawq.m")   % 绘制关节角
%run("drawEye_handError.m")
%run("drawTendonLength.m")