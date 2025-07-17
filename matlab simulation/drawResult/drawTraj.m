% 画出目标点在图像上的移动轨迹

% 画面中心
center=[u_des,v_des];
scatter(center(1),center(2),'mx',"SizeData",200,"LineWidth",2);hold on;
scatter(center(1),center(2),'mo',"SizeData",100,"LineWidth",2);hold on;
text(center(1)-80,center(2)+40,'Image center','Color','black','FontSize',15);hold on;
% % 以err_des为半径画圆
% para = [u_des-err_des, v_des-err_des, 2*err_des, 2*err_des];
% rectangle('Position', para, 'Curvature', [1 1],"EdgeColor","r","LineWidth",1,"LineStyle","--"); %画圆
% 目标点在图像上的初始位置
% scatter(state.ulist(1),state.vlist(1),'mx',"SizeData",200,"LineWidth",2);hold on;
% scatter(state.ulist(1),state.vlist(1),'mo',"SizeData",100,"LineWidth",2);hold on;
% text(state.ulist(1)-150,state.vlist(1)-10,'Initial position','Color','black','FontSize',15);hold on;

scatter(state.ulist1(1),state.vlist1(1),'mx',"SizeData",200,"LineWidth",2);hold on;
scatter(state.ulist1(1),state.vlist1(1),'mo',"SizeData",100,"LineWidth",2);hold on;
text(state.ulist1(1)-90,state.vlist1(1)+30,'Initial position','Color','black','FontSize',15);hold on;

scatter(state.ulist2(1),state.vlist2(1),'mx',"SizeData",200,"LineWidth",2);hold on;
scatter(state.ulist2(1),state.vlist2(1),'mo',"SizeData",100,"LineWidth",2);hold on;
text(state.ulist2(1)-90,state.vlist2(1)+30,'Initial position','Color','black','FontSize',15);hold on;
% for i=1:length(state.ulist)
%     u=state.ulist(i);
%     v=state.vlist(i);
%     scatter(u,v,"filled","MarkerFaceColor","red","SizeData",30);hold on;
%     % 标上序号
% %     text(u+3,v,num2str(i),'Color','black','FontSize',10);hold on;
% end

%w_tracking=scatter(state.ulist(1:state.k),state.vlist(1:state.k),"filled","MarkerFaceColor","red","SizeData",30);hold on;

w_tracking1=scatter(state.ulist1(1:state.k),state.vlist1(1:state.k),"red","SizeData",30);hold on;
w_tracking2=scatter(state.ulist2(1:state.k),state.vlist2(1:state.k),"green","SizeData",30);hold on;

grid on;box on;axis equal;
xlabel("u (pixel)"); xlim([0,camera.imageSize(1)]);
ylabel("v (pixel)"); ylim([0,camera.imageSize(2)]);
set(gca,'FontSize',15,'LineWidth',1.5); 


