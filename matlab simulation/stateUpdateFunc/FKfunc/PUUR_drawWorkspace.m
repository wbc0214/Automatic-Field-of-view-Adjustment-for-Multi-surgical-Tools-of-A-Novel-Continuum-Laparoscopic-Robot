% 蒙特卡洛法仿真机械臂可达工作空间
% 作者：张晶
% 日期：2024.01.01

run("bot_args.m");
L=bot.U_l*2+bot.disk_h*2+bot.Link_l;% 两个万向节中心的距离
cam_L=bot.U_l+bot.cam_l; % 第二个万向节中心到连续体机器人末端的距离

DH_Table=[% theta d   a   alpha
              0   0   0    pi;    % 向下
              0   0   0    pi/2;  % 平动关节
           pi/2   0   0    pi/2;  
              0   0   L      0;
             pi   0   0    pi/2;
          -pi/2   0   0    pi/2;
             pi cam_L 0      0;
];

%% 定义各个连杆以及关节类型，默认为转动关节
for i=1:size(DH_Table,1)
    Lk(i)=Link(DH_Table(i,:));
    if i==2
        Lk(i).jointtype='P';
        Lk(i).qlim=[bot.qlim(1,1),bot.qlim(2,1)];
    elseif i==1
        Lk(i).qlim=[0,1e-10];
    else
        Lk(i).qlim=[bot.qlim(1,i-1),bot.qlim(2,i-1)];
        Lk(i).offset=DH_Table(i,1);
    end
end
% 组合上述连杆
Mybot=SerialLink(Lk,'name','Mybot');

%% 使用蒙特卡洛法绘制机械臂的工作空间
N=15000;
q=rand(N,size(DH_Table,1))*(bot.qlim(2,2)-bot.qlim(1,2))+bot.qlim(1,2);
q(:,7)=rand(N,1)*(bot.qlim(2,6)-bot.qlim(1,6))+bot.qlim(1,6);
q(:,2)=rand(N,1)*bot.qlim(2,1);
q(:,1)=rand(N,1)*0; % 第一个关节不动
for n = 1:N
    workspace = Mybot.fkine(q(n,:));
    plot3(workspace.t(1),workspace.t(2),workspace.t(3),'b.','markersize',1);
    hold on;
end



q_rad=[0 0 0.1 0.1 0 0 0]; % 关节角度,rad
Mybot.plot(q_rad);
T_end_bot=Mybot.fkine(q_rad).T;
xlim([-50,150]);ylim([-100,100]);
xlabel("x (mm)");ylabel("y (mm)");zlabel("z (mm)");
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
box on; grid on;