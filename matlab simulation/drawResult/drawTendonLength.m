% 绘制绳长随时间的变化
% 作者：张晶

ID_list=["04","10","16","05","11","17","00","09"];
t=zeros(1,state.k);
for step=1:state.k-1
    t(step+1)=sum(state.duration(1:step));
end

for i=1:8
    subplot(4,2,i);
    plot(t,state.tendon_length(1:state.k,i)-state.tendon_length(1,i),'Color','r','LineWidth',1.5);hold on;
    title("Tendon"+ ID_list(i));
    xlabel("t (s)");
    ylabel("\Delta TL (mm)");ylim([0,80]);
    set(gca,'FontSize',13,'LineWidth',1,'xminortick','on','yminortick','on','YColor','k');
    grid on; box on;
end