
% 计算两个disk间同一孔洞间的距离
% 作者：张晶
% 日期：2023.12.30
% 输入
    % T1,T2：上下圆盘坐标系的齐次变换阵
    % bot：连续体机器人关节参数
        % bot.r_hole % 孔洞在disk上的分布半径，mm
        % bot.angle_hole % 孔洞间隔，rad
function l=get_length(T1,T2,bot,tendonID)

    T=TransInv(T1)*T2; % {2}坐标系在{1}中的表示
    hole=[bot.r_hole*cos(bot.angle_hole*tendonID);bot.r_hole*sin(bot.angle_hole*tendonID);0;1];% 在{1}或{2}中孔洞的齐次坐标
    l=norm(T*hole-hole);

end