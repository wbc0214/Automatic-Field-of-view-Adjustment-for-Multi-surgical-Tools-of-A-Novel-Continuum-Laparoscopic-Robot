% 绘制u、v误差与速度的关系
% 作者：张晶
% 日期：2023.12.20

% 横坐标： Convergence steps
% 纵坐标，左： Error/pixels
% 纵坐标，右：u_dot
steps=1:state.k;
figure();
subplot(211);
xlabel("Convergence steps","FontName","times new roman");

yyaxis left
plot(steps,state.pix_err(1,steps),'-x'); hold on;
ylabel("u Error/pixels","FontName","times new roman");

yyaxis right
plot(steps,state.u_dot(steps),'-x');
ylabel("Zc*u_dot/ pixels/s");

grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 

subplot(212);
xlabel("Convergence steps","FontName","times new roman");

yyaxis left
plot(steps,state.pix_err(2,steps),'-x'); hold on;
ylabel("v Error/pixels","FontName","times new roman");

yyaxis right
plot(steps,state.v_dot(steps),'-x');
ylabel("Zc*v_dot/ pixels/s");

grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 


figure();
for k=1:state.k-1
    [~,~,~,Jb]=PUUR_Screw(bot,state.qlist(k,:));
    Jb=[Jb(4:6,:);Jb(1:3,:)]; % v 在上
    q_dot_list(k,:)=(state.qlist(k+1,:)-state.qlist(k,:))/state.dt;
    V(:,k)=Jb*q_dot_list(k,:)';
end

subplot(121);

for i=1:3
    plot(1:state.k-1,V(i,:)); hold on;
end
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
legend(["vx","vy","vz"]);
xlabel("Convergence steps");


subplot(122);
for i=4:6
    plot(1:state.k-1,V(i,:)); hold on;
end
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
legend(["wx","wy","wz"]);
xlabel("Convergence steps");


figure();
subplot(211);
plot(steps,state.pix_err(1,steps),'Color','b','LineWidth',2); hold on;
ylabel("u Error/pixels","FontName","times new roman");
xlabel("Convergence steps","FontName","times new roman");
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 

subplot(212);
plot(steps,state.pix_err(2,steps),'Color','b','LineWidth',2); hold on;
xlabel("Convergence steps","FontName","times new roman");
ylabel("v Error/pixels","FontName","times new roman");
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 