clear
clc
q1=0; % 固定的转动关节
q2=10; %平动关节
q_deg=[0, 0, 0, 30, 0, 20, 0];
q_rad=q_deg*pi/180;
q_rad(2)=q2;

% % DH自己写的
% run("PUUR_z_T_end_DH.m")
% T_end
% 
% % DH robotic toolbox
% run("PUUR_z_drawWorkspace.m")
% T_end_bot

% 旋量理论
T_end_screw=PUUR_Screw(q_rad(2:end))