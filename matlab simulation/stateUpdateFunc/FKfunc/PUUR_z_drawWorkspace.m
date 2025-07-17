% clear;
%蒙特卡洛法仿真机械臂可达工作空间
%% 平动关节1+万向节2+旋转关节1

%% DH参数
L=100; % mm, length of the link
IN_MAX=60;
THETA_MAX=35*pi/180;
THETA_MIN=-35*pi/180;

DH_Table=[% theta d   a   alpha
              0   0   0     pi;
              0   0   0    pi/2;
           pi/2   0   0    pi/2;
              0   0   L      0;
             pi   0   0    pi/2;
          -pi/2   0   0    pi/2;
           pi/2   L   0     0;
               
];


%% 定义各个连杆以及关节类型，默认为转动关节
for i=1:size(DH_Table,1)
    Lk(i)=Link(DH_Table(i,:));Lk(i).qlim=[THETA_MIN,THETA_MAX];
end
Lk(2).jointtype='P';Lk(2).qlim=[0,IN_MAX];
Lk(3).offset=pi/2;
Lk(5).offset=pi;
Lk(6).offset=-pi/2;
Lk(7).offset=pi/2;Lk(7).qlim=[-pi,pi];
% q_rad=[0 0 0 0 0 0 0]; % 关节角度

% 组合上述连杆
bot=SerialLink(Lk,'name','bot');
% bot.display();


% %% 使用蒙特卡洛法绘制机械臂的工作空间
% N=15000;
% q=rand(N,size(DH_Table,1))*(THETA_MAX-THETA_MIN)+THETA_MIN;
% q(:,2)=rand(N,1)*IN_MAX;
% for n = 1:N
%     workspace = bot.fkine(q(n,:));
%     plot3(workspace.t(1),workspace.t(2),workspace.t(3),'b.','markersize',1);
%     hold on;
% end

T_end_bot=bot.fkine(q_rad).T;
bot.plot(q_rad);
% bot.teach();
