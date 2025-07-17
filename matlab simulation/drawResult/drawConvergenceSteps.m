% 绘制收敛过程（像素误差）
% 作者：张晶
% 日期：2023.11.28

% 横坐标： Convergence steps
% 纵坐标： Error/pixels
%fig_err=figure("Name","Error");
steps=1:state.k;
err=sqrt((state.vlist(steps)-v_des).^2+(state.ulist(steps)-u_des).^2);
% scatter(steps,err,'red','filled','SizeData',80);
% plot(steps,err,'-o','Color','b');hold on;
% plot(steps,err_des*ones(length(steps)),'--','Color','r',"LineWidth",2);hold on;
xlabel("Convergence steps");xlim([1,state.k]);
ylabel("Error (pixel)");
% legend(["Kp="+num2str(PID_param.Kp(1))+", Ki="+num2str(PID_param.Ki(1))+", Kd="+num2str(PID_param.Kd(1))]);
grid on;box on;
set(gca,'FontSize',10,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
set(gcf,'position',[540 540 600 400])


% 横坐标： t/s
% 纵坐标： Error/pixels
%figure();
t=zeros(1,state.k);
for step=1:state.k-1
    t(step+1)=sum(state.duration(1:step));
end
Tracking_error_threshold = plot(t,err_des*ones(length(t),1),"--",'Color','black','LineWidth',2);hold on;



b=plot(t,err,'Color','b','LineWidth',2);hold on;
%legend('Real tracking error','2D Line Traj',"FontName","times new roman","Location","best");
xlabel("t (s)","FontName","times new roman");
ylabel("Tracking error (pixel)","FontName","times new roman");
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on',"FontName","times new roman"); 
set(gcf,'position',[540 540 600 400])