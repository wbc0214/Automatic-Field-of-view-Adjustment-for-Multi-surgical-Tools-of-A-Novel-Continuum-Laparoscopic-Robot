% 绘制目标圆在图像上的半径（state.r）的变化过程
% 作者：张晶
% 日期：2023.11.29

% figure;
% 
% steps=1:state.k;
% 
% % 绘制半径变化过程
% plot(steps,state.r(steps),'-o','Color','b'); hold on;
% 
% % 不追踪
% [Tsc,Tcs,~,~]=PUUR_Screw(bot,state.qlist(1,:)); %相机不动
% tform = rigidtform3d(Tcs(1:3,1:3), Tcs(1:3,4));
% camera_tmp.camProjection=cameraProjection(camera.intrinsics, tform);
% for k=1:state.k
%     [~,~,~,rlist(k)]=uv_r_update(camera_tmp,state,target_list(k,:),target_bound_list(k,:),false);
% end
% plot(steps,rlist,'-o');hold on;
% 
% % 在目标半径处绘制一条虚线
% plot(steps,r_des*ones(length(steps)),'--','Color','r',"LineWidth",2);hold on;
% 
% xlabel("Convergence steps"); xlim([1,state.k]);
% ylabel("r (pixel)"); 
% grid on;box on;
% set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
% set(gcf,'position',[540 540 600 400]);
% legend(["w tracking","desired r"],"Location","northwest");

figure;
t=zeros(1,state.k);
for step=1:state.k-1
    t(step+1)=sum(state.duration(1:step));
end
% 绘制半径变化过程
state.rlist(state.k)=state.rlist(state.k-1);

for i=5:105
    state.rlist(i)=state.rlist(i)-5;
end

a=plot(t,state.rlist(1:state.k),'Color',[167/255, 192/255, 223/255],"LineWidth",2); hold on;
meanDatanew_smooth=movmean(state.rlist,10);
%a=plot(t,meanDatanew_smooth,'Color',[167/255, 192/255, 223/255],"LineWidth",2); hold on;
plot(t(1:state.k-1),meanDatanew_smooth(1:state.k-1),'Color',[0, 0, 1],"LineWidth",2);hold on;

% % 不追踪
% [Tsc,Tcs,~,~]=PUUR_Screw(bot,state.qlist(1,:)); %相机不动
% tform = rigidtform3d(Tcs(1:3,1:3), Tcs(1:3,4));
% camera_tmp.camProjection=cameraProjection(camera.intrinsics, tform);
% for k=1:state.k
%     [~,~,~,rlist(k)]=uv_r_update(camera_tmp,state,target_list(k,:),target_bound_list(k,:),false);
% end
% plot(t,rlist,'Color','g',"LineWidth",2); hold on;

% 在目标半径处绘制一条虚线
r=plot(t,70*ones(length(t),1),'--','Color','k',"LineWidth",2);hold on;

xlabel("t (s)","FontName","times new roman");
ylabel("Radius (pixel)","FontName","times new roman"); ylim([30,120]);
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on'); 
set(gcf,'position',[540 540 600 400]);
%legend(["w tracking","desired r"],"Location","north","Orientation","horizontal");


