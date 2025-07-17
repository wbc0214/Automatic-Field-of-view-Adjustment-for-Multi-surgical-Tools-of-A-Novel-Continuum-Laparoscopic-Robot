% 定义机械臂参数
bot.z0=100; % 初始时腹腔镜与底面的距离，mm，无论初始平动关节值为多少，z0均为该值
bot.U_l=13.6/2; % 半个万向节长度，mm，在solidworks中测量得到14.25mm，实际13.6mm
bot.Link_l=34; % link的长度（不计disk厚度），mm
bot.cam_l=50;% camera_holder长度（包含底面圆盘厚度），mm
bot.disk_h=3; % 圆片厚度，mm
bot.disk_r=20/2; % 圆片半径，mm
bot.r_hole=16/2; % 孔洞在disk上的分布半径，mm
bot.angle_hole=2*pi/18; % 孔洞间隔，rad
bot.rh=4; % R关节螺旋线半径，mm
bot.ph=3/(2*pi); % R关节螺旋线螺距/2pi，mm
q1lim=[20;60]; % 关节角约束，平动关节，mm
qlim=[-35*pi/180;35*pi/180]; % rad
bot.qlim=[q1lim,qlim,qlim,qlim,qlim,[-pi;pi]]; % rad
bot.dqlim=[2,0.01,0.01,0.01,0.01,0.1];
% 一个平动关节+两个万向节+一个link+一个camera_holder
bot.total_len=q1lim(2)+bot.U_l*4+bot.Link_l+bot.disk_h*3+bot.cam_l; % 机械臂长度，mm