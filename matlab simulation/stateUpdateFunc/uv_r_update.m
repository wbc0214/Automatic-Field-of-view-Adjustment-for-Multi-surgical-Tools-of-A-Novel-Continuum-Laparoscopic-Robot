% 更新目标圆中心在图像上的坐标，及半径
% 输入：
    % camera
        % camProjection：相机变换矩阵，含内外参
        % imageSize：图像大小
    % state:
        % Tsc：相机坐标系相对于世界坐标系的齐次变换阵
    % target：目标点在世界坐标系下的坐标
    % target_bound：目标圆圆周上一点在世界坐标系下的坐标
    % draw_camera_view：是否用drawCameraView3绘制结果
% 输出：
    % [u,v]：目标点的像素坐标
    % Zc：相机坐标系下的深度信息
    % r：目标圆在图像上的半径
% 作者：张晶
% 日期：2023.11.13
% 更新日志
    % 张晶2023.11.16：drawCameraView(T,VP)-->drawCameraView2(Tsc,target,camProjection,imageSize)
    % 张晶2023.11.23：drawCameraView2(T,VP)-->drawCameraView3(Tsc,target,camProjection,imageSize)
    % 张晶2023.11.28：copy from uv_update
    % 张晶2023.11.29：更新输入形式，在图像上绘制圆
    % 张晶2023.12.05：增加draw_camera_view
function [u,v,Zc,r]=uv_r_update(camera,state,target,target_bound,draw_camera_view)
    target_pix=camera.camProjection*[target,1]';
    u=target_pix(1)/target_pix(3);
    v=target_pix(2)/target_pix(3);
    Zc=target_pix(3);
    target_bound_pix=camera.camProjection*[target_bound,1]';
    r=norm(target_pix(1:2)/target_pix(3)-target_bound_pix(1:2)/target_bound_pix(3));
    if draw_camera_view
        drawCameraView3(state.Tsc,target,camera.camProjection,camera.imageSize)
        para = [u-r, v-r, 2*r, 2*r];
        rectangle('Position', para, 'Curvature', [1 1],"EdgeColor","r","LineWidth",1); %画圆
    end
end