% �����ʼ��
% ������־
    % �ž�2023.11.29�������������װ���ṹ��camera
    % �ž�2024.01.17���켣���ɵ�ǰʱ�����

% syms Zc u v
% % ĩ���ٶȵ�ͼ�������ٶȵ�ӳ��
% state.J=[
%      -fu/Zc,   0 ,   (u-u0)/Zc,  (u-u0)*(v-v0)/fv,  -fu-(u-u0)^2/fu,  fu*(v-v0)/fv;
%       0,   -fv/Zc,  (v-v0)/Zc,  fv+(v-v0)^2/fv,  -(u-u0)*(v-v0)/fu,  -fv*(u-u0)/fu;
%    ];

%% ��ʼ��������״̬
% ������Ҫ��list
state.ulist=zeros(1,1000);
state.vlist=zeros(1,1000);
state.ulist1=zeros(1,1000);
state.vlist1=zeros(1,1000);
state.ulist2=zeros(1,1000);
state.vlist2=zeros(1,1000);
state.r=zeros(1,1000);
state.r1=zeros(1,1000);
state.r2=zeros(1,1000);
state.pix_err=zeros(2,1000);
state.u_dot=zeros(1,1000);
state.v_dot=zeros(1,1000);
state.r_err=zeros(1,1000);
state.qlist=zeros(1000,6);
state.tendon_length=zeros(1000,8);
state.motor_angle=zeros(1000,9);
state.duration=zeros(1,1000);
% ��k=1
k=1;state.k=k;

state.Tse=[%eye
        1,  0,  0,  0;
        0, -1,  0,  0;
        0,  0, -1,  0;
        0,  0,  0,  1
       ];
state.dt=0.1;% ʱ����
state.Zclist(k)=100;% ����һ����ʼ��Zc��������£����ڵİ汾û���õ�
state.Zclist1(k)=100;
state.Zclist2(k)=100;
bot.z0=bot.z0+state.qlist(1,1);