% 绘制关节角的变化
% 作者：张晶
% 日期：2023.12.21
steps=1:state.k;
figure();
subplot(3,2,1);
plot(steps,state.qlist(steps,1),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q1/mm");
set(gca,'FontSize',15,'LineWidth',1.5); 

subplot(3,2,2);
plot(steps,state.qlist(steps,2),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q2/rad");
set(gca,'FontSize',15,'LineWidth',1.5); 

subplot(3,2,3);
plot(steps,state.qlist(steps,3),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q3/rad");
set(gca,'FontSize',15,'LineWidth',1.5); 

subplot(3,2,4);
plot(steps,state.qlist(steps,4),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q4/rad");
set(gca,'FontSize',15,'LineWidth',1.5); 

subplot(3,2,5);
plot(steps,state.qlist(steps,5),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q5/rad");
set(gca,'FontSize',15,'LineWidth',1.5); 

subplot(3,2,6);
plot(steps,state.qlist(steps,6),'LineWidth',1.5);
xlabel("Convergence steps");
ylabel("q6/rad");
set(gca,'FontSize',15,'LineWidth',1.5); 

% subplot(4,2,7);
% plot(steps,state.qlist(:,7),'LineWidth',1.5);
% xlabel("Convergence steps");
% ylabel("q7/rad");
% set(gca,'FontSize',15,'LineWidth',1.5); 
% 
% subplot(4,2,8);
% plot(steps,state.qlist(:,8),'LineWidth',1.5);
% xlabel("Convergence steps");
% ylabel("q8/rad");
% set(gca,'FontSize',15,'LineWidth',1.5); 

figure;
t=zeros(1,state.k);
for step=1:state.k-1
    t(step+1)=sum(state.duration(1:step));
end
N=5;
for i=1:N
    subplot(1,5,i);
    plot(t,state.qlist(steps,i+1),'Color','r',"LineWidth",2);hold on;
    xlabel("t (s)");
    ylbl=['q_',num2str(i),' (rad)'];
    ylabel(ylbl);
    set(gca,'FontSize',15,'LineWidth',1.5,'Fontname','Times new roman'); 
    ylim([-0.4,0.2]);
    xlim([0,T]);
    grid on;
end
