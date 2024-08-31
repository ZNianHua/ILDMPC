clear;clc;close all;
%% 参考路径
ka_test=40;kb_test=80;
k_a=100;k_b=10;
path_length=5000;
cx_test=1/path_length:1000/path_length:1000;
cy_test=cos(cx_test/kb_test).*cx_test/ka_test+sin(cx_test/ka_test).*cx_test/kb_test;
cx_last_test=[0,cx_test];
cy_last_test=[0,cy_test];
delta_x_test=cx_test(1:path_length)-cx_last_test(1:path_length);
delta_y_test=cy_test(1:path_length)-cy_last_test(1:path_length);
ctheta_test=atan2(delta_y_test,delta_x_test);
path_ref=[cx_test;cy_test;ctheta_test];
%% 仿真初始化
T_max=5000;Tstep=0.01;
Nx=8;
Nu=6;
ind_ref=1;
d0=10;%恒间距策略距离
T_follower=1;%跟随者启动时间
k=0.2;%纯跟踪预瞄增益
target_v=20*ones(1,T_max);%目标车速
N=1;%数据份数
state_vehice=zeros(Nx,T_max+1,N);
state_vehice(4,:,:)=(3+2*rand(1))*ones(1,T_max+1,N);
U=zeros(Nu,T_max,N);
phi=zeros(T_max,N);
r_tire=0.3;T_power_max=150;i0_all=4.1;eta_all=0.85;
M=1550;a=1.5;b=1.4;Ce=0.01;f=0.015;g=9.8;
Iz=1750;Cx=0.01;Cy=0.06;Cf=-90000;Cr=-70000;hg=0.6;tao=0.1;miu_f=1;miu_r=0;
Ca=0.2;tfl=0.8;tfr=0.8;trl=0.85;trr=0.85;Cfl=-45000;Cfr=-45000;Crl=-40000;Crr=-40000;
miufl=0.0155;miufr=0.0156;miurl=0.0151;miurr=0.0151;taux=0.09;tauy=0.1;
Rfl=0.31;Rfr=0.31;Rrl=0.32;Rrr=0.32;ifl=9;ifr=9;irl=8;irr=8;eta=0.85;g=9.8;

vehicle_param=[M;Iz;a;b;tfl;tfr;trl;trr;hg;Ca;Cfl;Cfr;Crl;Crr;...
    miufl;miufr;miurl;miurr;taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
vehicle_param_ref=[M;Iz;a;b;tfl;tfr;trl;trr;hg;0;Cfl;Cfr;Crl;Crr;...
    0;0;0;0;0.01;0.01;0.3;0.3;0.3;0.3;ifl;ifr;irl;irr;1;g];
%% 仿真过程
for j=1:N
    for i=1:T_max
%         phi(i,j)=0.05*cos(state_vehice(1,i,j)/k_a)+0.05*sin(state_vehice(2,i,j)/k_b);
        phi(i,j)=0;
        %% 纯跟踪算法测试+纯跟踪算法参考生成
        Ld=k*state_vehice(4,i,j)+0.1;
        [ind_ref,point_ref] = calc_target_index(state_vehice(:,i,j),path_ref,Ld,ind_ref);%找最近点
        [delta] = pure_pursuit_control(state_vehice(:,i,j),point_ref,a+b,Ld);%纯跟踪
        T_drive=70*(target_v(i)-state_vehice(4,i,j));
        U(:,i,j)=[T_drive/4;T_drive/4;T_drive/4;T_drive/4;delta;delta];
        state_vehice(:,i+1,j)= VehicleModel_DistributedDrive(state_vehice(:,i,j),vehicle_param_ref(:,j),U(:,i,j),phi(i,j),Tstep);
    end      
end
%% 状态画图
num_row=2;
num_colum=4;
title_name_part2={'x-cooridinate','y-cooridinate','YAW angle','longitudinal velocity','lateral velocity','YAW rate','ax','ay'};
figure
for k=1:num_row*num_colum
    subplot(num_row,num_colum,k)
    plot(state_vehice(k,2:end));
    title(['reference ',title_name_part2{k}])
end
%% 数据保存
% start_t=500;
% end_t=5000;
% recorded_reference=state_vehice(:,start_t:end_t);
% save('reference.mat','recorded_reference');