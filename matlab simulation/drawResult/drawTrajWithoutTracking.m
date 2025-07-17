% 绘制without tracking的情况下，目标点在图像上的移动
% 作者：张晶
% 日期：2023.11.29

ulist=zeros(state.k,1);
vlist=zeros(state.k,1);
ulist1=zeros(state.k,1);
vlist1=zeros(state.k,1);
ulist2=zeros(state.k,1);
vlist2=zeros(state.k,1);
[Tsc,Tcs,~,~]=PUUR_Screw(bot,state.qlist(1,:)); %相机不动
tform = rigidtform3d(Tcs(1:3,1:3), Tcs(1:3,4));
camProjection=cameraProjection(camera.intrinsics, tform);
for i=1:state.k
    if target_num == 1
        target_pix=camProjection*[target_list(i,:),1]';
        ulist(i)=target_pix(1)/target_pix(3);
        vlist(i)=target_pix(2)/target_pix(3);
    else
        target_pix1=camProjection*[target_list1(i,:),1]';
        target_pix2=camProjection*[target_list2(i,:),1]';
        ulist1(i)=target_pix1(1)/target_pix1(3);
        vlist1(i)=target_pix1(2)/target_pix1(3);
        ulist2(i)=target_pix2(1)/target_pix2(3);
        vlist2(i)=target_pix2(2)/target_pix2(3);
    end
end

% wo_tracking=scatter(ulist,vlist,"filled","MarkerFaceColor","green","SizeData",30);
% legend([w_tracking,wo_tracking],["w tracking","w/o tracking"])

wo_tracking1=scatter(ulist1,vlist1,"filled","MarkerFaceColor","red","SizeData",30);
wo_tracking2=scatter(ulist2,vlist2,"filled","MarkerFaceColor","green","SizeData",30);
legend([w_tracking1,w_tracking2,wo_tracking1,wo_tracking2],["w tracking1","w tracking2","w/o tracking1","w/o tracking2"])
