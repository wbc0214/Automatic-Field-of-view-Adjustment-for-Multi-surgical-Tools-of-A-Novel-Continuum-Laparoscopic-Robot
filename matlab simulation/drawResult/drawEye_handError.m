% 手眼一致性
xc=[1;0;0];
zc=[0;0;1];
xe=[1;0;0];
for k=1:state.k
    qk=state.qlist(k,:);
    [Tsc,~,~,~]=PUUR_Screw(bot,qk);
    Tec=TransInv(state.Tse)*Tsc;
    Rec=Tec(1:3,1:3);
    % 在眼睛坐标系下
    n=cross(xe,Rec*zc); % 平面法向量
    xc_ine=Rec*xc;
    n_cross_xc_ine=cross(n,xc_ine);
    % 计算n与xc_ine的夹角，范围为-pi到pi
    if n_cross_xc_ine(3)>0
        theta=atan2d(-norm(cross(n,xc_ine)),dot(n,xc_ine));% deg
    else
        theta=atan2d(norm(cross(n,xc_ine)),dot(n,xc_ine));% deg
    end


    % 求平面与xc_ine的夹角
    if theta>-180 && theta<=90
        state.eye_hand_err(k)=theta+90;% deg
    else
        state.eye_hand_err(k)=theta-270;% deg
    end
    
    
end
%figure()
t=zeros(1,state.k);
for step=1:state.k-1
    t(step+1)=sum(state.duration(1:step));
end
c=plot(t,state.eye_hand_err(1:state.k),'Color','b',"LineWidth",2);hold on;
xlabel("t (s)","FontName","times new roman");
ylabel("Eye-hand error (degree)","FontName","times new roman");ylim([-10,10]);
grid on;box on;
set(gca,'FontSize',15,'LineWidth',1.5,'xminortick','on','yminortick','on',"FontName","times new roman"); 
set(gcf,'position',[540 540 600 400])