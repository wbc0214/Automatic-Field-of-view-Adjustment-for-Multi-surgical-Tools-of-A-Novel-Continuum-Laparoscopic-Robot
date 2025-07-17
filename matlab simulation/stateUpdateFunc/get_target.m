% target位置由当前时间决定
% 作者：张晶
% 日期：2024.01.17

% 输入
    % traj_shape：轨迹形状
    % T：总时间
    % t：当前时间
% 输出
    % target：目标位置
    % target_bound：目标圆圆周上一点，用以计算半径
    % traj_shape_list：可供选择的轨迹形状列表
    % target_num：器械数量
% reference
    % [1] J. Peng, C. Zhang, L. Kang, and G. Feng, “Endoscope FOV Autonomous Tracking Method for Robot-Assisted Surgery Considering Pose Control, Hand–Eye Coordination, and Image Definition,” IEEE Trans. Instrum. Meas., vol. 71, pp. 1–16, 2022, doi: 10.1109/TIM.2022.3204086.

% 更新日志
    % 张晶2024.04.02：增加2个器械的场景
    % 张晶2024.04.22：输出器械数量


function [target,target_bound,traj_shape_list,target_num] = get_target(traj_shape,T,t)
    traj_shape_list=[
        "point",... % 单个固定点
        "points",... % 多个固定点
        "line2D",... % 2D直线
        "line3D",... % 3D直线
        "circle2D",... % 2D圆
        "circle3D",... % [1] eq(31)
        "cut",... % 剪切
        "ablation",... % 消融
        "two_points"
    ];
    target_num = 1;
    switch traj_shape
        case "point"
            % target和target_bound均为固定点
            target=[50,30,0];
            target_bound=[60,30,0];
        case "points"
            target_list = [25, 0, 0;
                          50, 0, 0;
                          25, 25, 0;
                           0,  0, 0;
                          25, -25, 0];
            target_bound_list = [25, -10, 0;
                          50, -10, 0;
                          25, 15, 0;
                           0,-10, 0;
                          25, -35, 0];

            target_num = size(target_list, 1);
            segment_size = T / target_num;
            segment_index = floor(t / segment_size) + 1;
            if segment_index > target_num
                segment_index = target_num;
            end
            target = target_list(segment_index, :);
            target_bound = target_bound_list(segment_index, :);
        case "line2D"
            start_point=[0,0,0];
            end_point=[80,0,0];
            target=start_point+(end_point-start_point)*t/T;
            start_point=[0,10,0];
            end_point=[80,10,0];
            target_bound=start_point+(end_point-start_point)*t/T;
        case "line3D"
            start_point=[-20,0,0];
            end_point=[100,0,50];
            target=start_point+(end_point-start_point)*t/T;
            start_point=[-20,10,0];
            end_point=[100,10,50];
            target_bound=start_point+(end_point-start_point)*t/T;
        case "circle2D"
            c_center=[30,0,0]; % 轨迹圆中心，mm
            c_r=30; % 轨迹圆半径，mm
            target=c_center+[c_r*cos(2*pi*t/T+pi/2),c_r*sin(2*pi*t/T+pi/2),0];
            target_bound=c_center+[(c_r+10)*cos(2*pi*t/T+pi/2),(c_r+10)*sin(2*pi*t/T+pi/2),0];
        case "circle3D"
            r=50; % mm
            h=20; % mm
            h0=10; % mm
            target=[r*cos(2*pi*t/T),r*sin(2*pi*t/T),h0+h*sin(2*pi*t/T)];
            target_bound=[(r+10)*cos(2*pi*t/T),(r+10)*sin(2*pi*t/T),h0+h*sin(2*pi*t/T)];
        case "cut"
            % 器械1沿直线运动
            start_point=[-10,-10,0];
            end_point=[100,-10,0];
            target_1=start_point+(end_point-start_point)*t/T;
            target_list = [-10, 20, 0;
                             8, 20, 0;
                            26, 20, 0;
                            44, 20, 0;
                            50, 20, 0];
            target_num = size(target_list, 1);
            segment_size = T / target_num;
            segment_index = floor(t / segment_size) + 1;
            if segment_index > target_num
                segment_index = target_num;
            end
            target_2 = target_list(segment_index, :);
            target = [target_1; target_2];
            target_bound = [target_1; target_2]+[0,10,0;0,10,0]; % 先赋个值防止报错
            target_num = 2;
        case "ablation"
            start_point=[-10,-10,0];
            mid_point=[20,-10,0];
            end_point=[50,-10,0];
            if t>T*3/4
                target_1=end_point+(mid_point-end_point)*(t-T*3/4)*4/T;
                target_2=[35, 20, 0];
            elseif t>T/2
                target_1=mid_point+(end_point-mid_point)*(t-T/2)*4/T;
                target_2=[35, 20, 0];
            elseif t>T/4
                target_1=mid_point+(start_point-mid_point)*(t-T/4)*4/T;
                target_2=[5, 20, 0];
            else
                target_1=start_point+(mid_point-start_point)*t*4/T;
                target_2=[5, 20, 0];
            end
            target=[target_1; target_2];
            target_bound=[target_1; target_2]+[0,10,0;0,10,0]; % 先赋个值防止报错
            target_num = 2;
        case "suture"
            if t<5
                target_1=([10,10,0]-[0,0,0])*t/5+[0,0,0];
                target_2=[5,-5,0];
                target_num = 2;
            elseif t<7.5
                target_1=([100,10,0]-[10,10,0])*(t-5)/2.5+[10,10,0];
                target_2=[5,-5,0];
                target_num = 2;
            elseif t<10
                target_1=([10,5,0]-[100,10,0])*(t-7.5)/2.5+[100,10,0];
                target_2=[5,-5,0];
                target_num=2;
            else
                target_1=[10,5,0];
                target_2=([-20,5,0]-[5,-5,0])*(t-10)/5+[5,-5,0];
                target_num = 2;
            end
            if target_num == 1
                target=[target_2];
                target_bound=[target_2]; % 先赋个值防止报错
            else
                target=[target_1; target_2];
                target_bound=[target_1; target_2]; % 先赋个值防止报错
            end
        case "two_points"
            target_1=[10,40,0];
            target_2=[40,40,0];
            target=[target_1; target_2];
            target_bound=[target_1; target_2]+[0,10,0;0,10,0]; % 先赋个值防止报错
            target_num = 2;
        case "two_circle"
            c_center1=[30,0,0]; % 轨迹圆中心，mm
            c_center2=[0,0,0];
            c_r=30; % 轨迹圆半径，mm
            target_1=c_center1+[c_r*cos(2*pi*t/T+pi/2),c_r*sin(2*pi*t/T+pi/2),0];
            target_bound_1=c_center1+[(c_r+10)*cos(2*pi*t/T+pi/2),(c_r+10)*sin(2*pi*t/T+pi/2),0];
            target_2=c_center2+[c_r*cos(2*pi*t/T+pi/2),c_r*sin(2*pi*t/T+pi/2),0];
            target_bound_2=c_center2+[(c_r+10)*cos(2*pi*t/T+pi/2),(c_r+10)*sin(2*pi*t/T+pi/2),0];
            target=[target_1; target_2];
            target_bound=[target_bound_1;target_bound_2];
            target_num = 2;

    end
end