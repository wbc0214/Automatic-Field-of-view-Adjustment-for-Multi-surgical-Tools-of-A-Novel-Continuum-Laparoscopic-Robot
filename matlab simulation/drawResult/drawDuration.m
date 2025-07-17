% 绘制单步计算时长（state.duration)
% 作者：张晶
% 日期：2023.12.1

figure;
steps=1:state.k-1;
plot(steps,state.duration(1:state.k-1),'-o','Color','b');
xlabel("Convergence steps"); xlim([1,state.k]);
ylabel("Time per step/s");
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
set(gcf,'position',[540 540 600 400])