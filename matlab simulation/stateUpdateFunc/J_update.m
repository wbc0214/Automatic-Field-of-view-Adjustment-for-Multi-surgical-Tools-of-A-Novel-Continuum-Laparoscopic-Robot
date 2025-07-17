% 更新末端速度到图像特征速度的映射J
% 输入
    % camera：相机参数
        % fu,fv: 焦距 pix
        % u0,v0：光轴中心
    % state：
        % u,v：目标点在图像上的像素坐标
        % Zc：目标点在相机坐标系下的深度信息

% 输出
    % 当前状态下末端速度到图像特征速度的映射J
% 作者：张晶
% 日期：2023.11.23
% reference
    % [1] 徐文福，机器人学：基础理论及应用实践 p335-336
    % [2] C. Zhang, W. Zhu, J. Peng, Y. Han, and W. Liu, “Visual servo control of endoscope-holding robot based on multi-objective optimization: System modeling and instrument tracking,” Measurement, vol. 211, p. 112658, Apr. 2023, doi: 10.1016/j.measurement.2023.112658.

% 更新日志
    % 张晶2023.11.29：将输入更新为camera,state,Zc，深度估计过程中Zc是变量
    % 张晶2023.12.06：仅做三维更新
function J=J_update(camera,state,Zc)
    fu=camera.fu; fv=camera.fv; u0=camera.u0; v0=camera.v0;
    u=state.ulist(state.k); v=state.vlist(state.k);
%     J=[
%      -fu/Zc,   0 ,   (u-u0)/Zc,  (u-u0)*(v-v0)/fv,  -fu-(u-u0)^2/fu,  fu*(v-v0)/fv;
%       0,   -fv/Zc,  (v-v0)/Zc,  fv+(v-v0)^2/fv,  -(u-u0)*(v-v0)/fu,  -fv*(u-u0)/fu;
%     ];
    % ref [2] eq(7)
    J=[
     -fu,   0 ,   (u-u0);
      0,   -fv,   (v-v0)
    ];
%     J=[
%      -fu,   0 ,   0;
%       0,   -fv,   0
%     ];
%     J=[
%      -fu,   0 ;
%       0,   -fv
%     ];
   
end