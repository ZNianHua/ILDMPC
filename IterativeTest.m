
clear;clc;close all;
%% 参考路径
R_path=100;x_curve=300;ds=0.1;
slope_factor=0.1;slope1_gain=abs(rand(1))*slope_factor;slope2_gain=abs(rand(1))*slope_factor;
cx_part1=ds:ds:x_curve;
cy_part1=0.*cx_part1+R_path;
t_part2=0:ds/R_path:pi;
cx_part2=x_curve+R_path.*sin(t_part2);
cy_part2=R_path.*cos(t_part2);
cx_part3=x_curve:-ds:-x_curve;
cy_part3=0.*cx_part3-R_path;
t_part4=pi:ds/R_path:2*pi;
cx_part4=-x_curve+R_path.*sin(t_part4);
cy_part4=R_path.*cos(t_part4);
cx_part5=-x_curve:ds:0;
cy_part5=0.*cx_part5+R_path;
cx=[cx_part1,cx_part2,cx_part3,cx_part4,cx_part5];
cy=[cy_part1,cy_part2,cy_part3,cy_part4,cy_part5];
[s_withoutz] = CalculateStationSequence([cx;cy]);
cz_zero=0.*cx;total_length=length(cx);
slope1_startindex=floor(length(cx)/8);
slope1_endindex=ceil(length(cx)/4);
cz_slope1_ori=slope1_gain.*s_withoutz(slope1_startindex:slope1_endindex);
cz_slope1=cz_slope1_ori-cz_slope1_ori(1);
slope2_startindex=floor(2*length(cx)/3);
slope2_endindex=ceil(7*length(cx)/8);
cz_slope2_ori=slope2_gain.*s_withoutz(slope2_startindex:slope2_endindex);
cz_slope2=cz_slope2_ori-cz_slope2_ori(1)+cz_slope1(end);
cz=[cz_zero(1:slope1_startindex),cz_slope1,cz_slope1(end).*ones(1,slope2_startindex-slope1_endindex-1),cz_slope2,cz_slope2(end).*ones(1,total_length-slope2_endindex-1)];
[s_withz] = CalculateStationSequence([cx;cy;cz]);
ctheta=[0.*cx_part1,-t_part2,0.*cx_part3-pi,-t_part4,0.*cx_part5-2*pi];
cphi=[zeros(1,slope1_startindex),0.*cz_slope1+slope1_gain,zeros(1,slope2_startindex-slope1_endindex-1),0.*cz_slope2+slope2_gain,zeros(1,total_length-slope2_endindex-1)];
curvarure=[0.*cx_part1,-1/R_path+0.*cx_part2,0.*cx_part3,-1/R_path+0.*cx_part4,0.*cx_part5];
reference_path=[cx;cy;ctheta;curvarure;cz;cphi;s_withz];
torlelength=60;loading_start=ceil(11*length(s_withz)/24);loading_end=floor(13*length(s_withz)/24);
%% 仿真初始化
T_max=30000;Tstep=0.01;
Nx=8;Nx_frenet=3;Nx_road=2;N_nonlinearconstrain=8;Nu=2;
d_s=10;%恒间距策略距离
T_follower=1;%跟随者启动时间
k=0.2;%纯跟踪预瞄增益
transport_v=24;loading_v=24;%目标车速
N=4;%编队数量
targer_index=ones(1,N);
ind_ref=ones(T_max+1,N);
cover_time=ones(1,N);
state_vehicle=zeros(Nx,T_max+1,N);
state_model_predicted=zeros(Nx,T_max+1,N);
state_vehicle_frenet=zeros(Nx_frenet,T_max+1,N);
history_cost=zeros(T_max,N);
road_condition=zeros(Nx_road,T_max,N);
for ind_intial=1:N
    state_vehicle(1,:,ind_intial)=((N-ind_intial+0.5)*d_s+2*rand(1))+state_vehicle(1,:,ind_intial);
    ind_ref(1,ind_intial)=max(1,round(state_vehicle(1,1,ind_intial)/ds));
end
state_vehicle(2,:,:)=(R_path-1+2*rand(1))*ones(1,T_max+1,N);
state_vehicle(3,:,:)=(-0.1+0.2*rand(1))*ones(1,T_max+1,N);
state_vehicle(4,:,:)=(3+2*rand(1))*ones(1,T_max+1,N);
U=zeros(Nu,T_max,N);
dU=zeros(Nu,T_max,N);
r_tire=0.3;T_power_max=150;i0_all=4.1;eta_all=0.85;
M=1550;a=1.5;b=1.4;Ce=0.01;f=0.015;g=9.8;
Iz=1750;Cx=0.01;Cy=0.06;Cf=-90000;Cr=-70000;hg=0.6;tao=0.1;miu_f=1;miu_r=0;
Ca=0.2;tfl=0.8;tfr=0.8;trl=0.85;trr=0.85;Cfl=-45000;Cfr=-45000;Crl=-40000;Crr=-40000;
miufl=0.0155;miufr=0.0156;miurl=0.0151;miurr=0.0151;taux=0.09;tauy=0.1;
Rfl=0.31;Rfr=0.31;Rrl=0.32;Rrr=0.32;ifl=9;ifr=9;irl=8;irr=8;eta=0.85;g=9.8;

load('iteration0.mat')
state_recorded_XOY=final_database.VehicleStatesXOY;
rand_state=[1;1;0.1;1;1;0.1;1].*0.1;
if length(rand_state)~=Nx
    rand_state=[rand_state;zeros(Nx-length(rand_state),1)];
end
for ind_intial=1:N
    state_vehicle(:,1,ind_intial)=(2*rand(1)-1)*rand_state+state_recorded_XOY(:,1,ind_intial);
    ind_ref(ind_intial)=max(1,round( (state_vehicle(1,1,ind_intial)/ds) ));
end


% vehicle_param1=[1270;1536.7;1.015;1.895;0.837;0.837;0.837;0.837;0.54;0.3*(0.95+0.1*rand(1));-69500;-69500;-48000;-48000;...
%     0.0117;0.0117;0.0117;0.0117;
%     0.086;0.1*(0.9+0.2*rand(1));0.325;0.325;0.325;0.325;
%     2*4.1;2*4.1;0;0;0.88;9.8;350];
% vehicle_param2=[1110;1343.1;1.04;1.56;0.74;0.74;0.742;0.742;0.54;0.3*(0.95+0.1*rand(1));-46500;-46500;-34500;-34500;...
%     0.0134;0.0134*(0.95+0.1*rand(1));0.0134*(0.95+0.1*rand(1));0.0134*(0.95+0.1*rand(1));
%     0.085;0.1*(0.9+0.2*rand(1));0.298;0.298;0.298;0.298;
%     2*4.1;2*4.1;0;0;0.85;9.8;300];
% vehicle_param3=[1370;2315.3;1.11;1.756;0.775;0.775;0.775;0.775;0.52;0.3*(0.95+0.1*rand(1));-70000;-70000;-48500;-48500;...
%     0.0117;0.0117;0.0117;0.0117;
%     0.086;0.1*(0.9+0.2*rand(1));0.325;0.325;0.325;0.325;
%     2*4.1;2*4.1;0;0;0.9;9.8;460];
% vehicle_param4=[1430;2059.2;1.05;1.61;0.782;0.782;0.782;0.782;0.65;0.3*(0.95+0.1*rand(1));-50000;-50000;-39000;-39000;...
%     0.0121;0.0121;0.0121;0.0121;
%     0.089;0.1*(0.95+0.1*rand(1));0.347;0.347;0.347;0.347;
%     4.1;4.1;4.1;4.1;0.85;9.8;470];
% vehicle_param=[vehicle_param1,vehicle_param2,vehicle_param3,vehicle_param4];
% vehicle_param_ori=[M;Iz;a;b;tfl;tfr;trl;trr;hg;Ca;Cfl;Cfr;Crl;Crr;...
%     miufl;miufr;miurl;miurr;taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
% variable_gain1=1+0.1*rand(1);
% variable_gain2=1+0.1*rand(1);
% variable_gain3=1+0.1*rand(1);
% variable_gain4=1+0.1*rand(1);
% vehicle_param1=[variable_gain1*M;variable_gain1*Iz;variable_gain1*a;b;tfl;tfr;trl;trr;hg;Ca;variable_gain1*Cfl;variable_gain1*Cfr;Crl;Crr;...
%     variable_gain1*miufl;variable_gain1*miufr;miurl;miurr;variable_gain1*taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
% vehicle_param2=[variable_gain2*M;variable_gain2*Iz;variable_gain2*a;b;tfl;tfr;trl;trr;hg;Ca;variable_gain2*Cfl;variable_gain2*Cfr;Crl;Crr;...
%     variable_gain2*miufl;variable_gain2*miufr;miurl;miurr;variable_gain2*taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
% vehicle_param3=[variable_gain3*M;variable_gain3*Iz;variable_gain3*a;b;tfl;tfr;trl;trr;hg;Ca;variable_gain3*Cfl;variable_gain3*Cfr;Crl;Crr;...
%     variable_gain3*miufl;variable_gain3*miufr;miurl;miurr;variable_gain3*taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
% vehicle_param4=[variable_gain4*M;variable_gain4*Iz;variable_gain4*a;b;tfl;tfr;trl;trr;hg;Ca;variable_gain4*Cfl;variable_gain4*Cfr;Crl;Crr;...
%     variable_gain4*miufl;variable_gain4*miufr;miurl;miurr;variable_gain4*taux;tauy;Rfl;Rfr;Rrl;Rrr;ifl;ifr;irl;irr;eta;g];
% vehicle_param=[vehicle_param1,vehicle_param2,vehicle_param3,vehicle_param4];
load('VehicleParam.mat');
%% MPC参数
No=2;Np=1;Nc=1;h=0.3;vel_acc_torler_percent=0.1;
MPC_param.ErrorThres=0.01;%误差百分比
MPC_param.PredHori=Np;%预测域
MPC_param.ContrHori=Nc;%控制域
MPC_param.StateLengt=Nx;%状态向量维度
MPC_param.ContrLengt=Nu;%控制向量维度
MPC_param.OtherLengt=No;%其他输入维度
MPC_param.SamplTime=Tstep;%时间步长

MPC_param.WeiPerfo=diag([1000,1000,10000,100,1,5000,50,1]);%跟踪效果权重矩阵
MPC_param.WeiIncre=diag([1,10000]);%控制增量权重矩阵
MPC_param.WeiSimilJudg=diag([1,1,100,1,1,100,1,1]);%计算终端代价时，计算当前状态与历史状态相对差异所用权重
MPC_param.WeiTrans =diag(1);%运输前进权重

MPC_param.InterVehDis=d_s;%编队固定车间距
MPC_param.SafeVehDis=h;%编队安全时距
MPC_param.EndStati=s_withz(end);%终点s坐标
MPC_param.MinNum=5;%计算终端代价时，判断当前状态与历史状态集中点重合过程所取的前n个历史状态
MPC_param.ThresSimilJudg=0.001;%计算终端代价时，判断当前状态与历史状态集中点重合过程相对差异阈值
MPC_param.InfRepla=1e10;%计算终端代价时，无穷大的替代值

MPC_param.AccLgtMax=7;
MPC_param.AccLatMax=4;
MPC_param.LatDisMax=3;
MPC_param.LatDisMin=-3;
MPC_param.TermCostAct=0;%终端代价启用使能
MPC_param.PreStatiTimeHead=0.1;%为领航车准备的前向预瞄时距
MPC_param.AxBoundTolerPosi=vel_acc_torler_percent*MPC_param.AccLgtMax;%纵向加速度越界惩罚区间
MPC_param.AyBoundTolerPosi=vel_acc_torler_percent*MPC_param.AccLatMax;%横向加速度越界惩罚区间
MPC_param.AxBoundTolerNega=-MPC_param.AxBoundTolerPosi;%纵向加速度越界惩罚区间
MPC_param.AyBoundTolerNega=-MPC_param.AyBoundTolerPosi;%横向加速度越界惩罚区间
MPC_param.LatDistBoundTolerPosi=vel_acc_torler_percent*MPC_param.LatDisMax;%纵向加速度越界惩罚区间
MPC_param.LatDistBoundTolerNega=vel_acc_torler_percent*MPC_param.LatDisMin;%横向加速度越界惩罚区间
MPC_param.PnishScalFactr=20;
nonlinear_constrains=zeros(N_nonlinearconstrain,T_max,N);
end_pre_traj=ones(1,N);
%% 仿真过程
Gain_LoadSca=30/loading_v;
Gain_delta=[0,0,-0.03,-0.3,0.1,0];
Gain_drive=[50,500,0,0,0,100];
tic
for j=1:N
    for i=1:T_max
        
        MPC_param.TMax=vehicle_param(end,j);%最大扭矩
        MPC_param.DeltaMax=0.6;%最大转角
        MPC_param.DeltaTMax=MPC_param.TMax*Tstep;%最大扭矩增量
        MPC_param.DeltaDeltaMax=MPC_param.DeltaMax*Tstep;%最大转角增量
        MPC_param.TMin=-MPC_param.TMax;%最小扭矩
        MPC_param.DeltaMin=-0.6;%最小转角
        MPC_param.DeltaTMin=MPC_param.TMin*Tstep;%最小扭矩增量
        MPC_param.DeltaDeltaMin=MPC_param.DeltaMin*Tstep;%最小转角增量
        Tq_max=MPC_param.TMax;%最大扭矩
        delta_max=MPC_param.DeltaMax;%最大转角
        ub1=MPC_param.DeltaTMax;%最大扭矩增量
        ub2=MPC_param.DeltaDeltaMax;%最大转角增量
        Tq_min=MPC_param.TMin;%最小扭矩
        delta_min=MPC_param.DeltaMin;%最小转角
        lb1=MPC_param.DeltaTMin;%最小扭矩增量
        lb2=MPC_param.DeltaDeltaMin;%最小转角增量
        %上下界约束
        UB_vector=repmat([ub1;ub2],Nc,1);
        LB_vector=repmat([lb1;lb2],Nc,1);
        %% XOY2Frenet
        Ld=k*state_vehicle(4,i,j)+0.1;
        [state_vehicle_frenet(:,i,j),road_condition(:,i,j),ind_ref(i+1,j),~] = VehicleLocationOnFrenet(state_vehicle(:,i,j),reference_path,Ld,ind_ref(i,j));
        if state_vehicle_frenet(1,i,j)>(s_withz(loading_start)-torlelength) && state_vehicle_frenet(1,i,j)<(s_withz(loading_end)+torlelength)
            MPC_param.TransTargtVel=loading_v;
        else
            MPC_param.TransTargtVel=transport_v;
        end      
        if state_vehicle_frenet(1,i,j)>(s_withz(loading_start)) && state_vehicle_frenet(1,i,j)<(s_withz(loading_end))
            MPC_param.TransMaxVel=loading_v*Gain_LoadSca;
            MPC_param.VxBoundTolerPosi=vel_acc_torler_percent*MPC_param.TransMaxVel;%纵向速度越界惩罚区间
        else
            MPC_param.TransMaxVel=transport_v*Gain_LoadSca;
            MPC_param.VxBoundTolerPosi=vel_acc_torler_percent*MPC_param.TransMaxVel;%纵向速度越界惩罚区间
        end
        MPC_param.TransMinVel=0;
        MPC_param.VxBoundTolerNega=MPC_param.VxBoundTolerPosi;
        %线性约束
        if i==1
            input_last=U(:,i,j);
            state_model_predicted(:,i,j)=[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)];
        else
            input_last=U(:,i-1,j);
        end
        state_model_predicted(:,i,j)=[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)];
        u_UB_vector=repmat([Tq_max;delta_max],Nc,1);
        u_LB_vector=repmat([Tq_min;delta_min],Nc,1);
        b1=u_UB_vector-repmat(input_last,Nc,1);
        A1=kron(tril(ones(Nc)),ones(Nu,1));
        b2=repmat(input_last,Nc,1)-u_LB_vector;
        A2=-A1;
        b=[b1;b2];
        A=[A1;A2];
        
        if j==1
            velocity_error=MPC_param.TransTargtVel-state_vehicle(4,i,j);
%             velocity_error=0;
            station_error=0;
            omega_error=state_vehicle(4,i,j).*road_condition(2,i,j)/cos(road_condition(1,i,j))-state_vehicle(6,i,j);
            a_error=0-state_vehicle(7,i,j);
            error_state=[station_error;velocity_error;state_vehicle_frenet(2:3,i,j);omega_error;a_error];
        else
%             d0=d_s+h*state_vehicle(4,i,j);
            d0=d_s;
%             end_pre_traj(j)=max([targer_index(j-1),min(i,cover_time(j-1)),end_pre_traj(j)+1]);
            end_pre_traj(j)=min(i,cover_time(j-1));
            [~,min_index]=min(abs(state_model_predicted(1,1:end_pre_traj(j))-state_model_predicted(1,end_pre_traj(j))+d0));
%             velocity_error=state_vehicle(4,i,j-1)-state_vehicle(4,i,j);
            velocity_error=state_vehicle(4,min_index,j-1)-state_vehicle(4,i,j);
            station_error=state_vehicle_frenet(1,i,j-1)-state_vehicle_frenet(1,i,j)-d0;
            omega_error=state_vehicle(4,i,j).*road_condition(2,i,j)/cos(road_condition(1,i,j))-state_vehicle(6,i,j);
            a_error=state_vehicle(7,i,j-1)-state_vehicle(7,i,j);
            error_state=[station_error;velocity_error;state_vehicle_frenet(2:3,i,j);omega_error;a_error];
%             error_state=[station_error;velocity_error;state_vehicle_frenet(2,i,j)-state_vehicle_frenet(2,min_index,j-1);state_vehicle_frenet(3,i,j)];
        end

        delta_ori=Gain_delta*error_state;
        delta_abs=min(abs(delta_ori),delta_max);
        delta=sign(delta_ori)*delta_abs;
        T_ori=Gain_drive*error_state;
        T_abs=min(abs(T_ori),Tq_max);
        T_drive=sign(T_ori)*T_abs;
        U(:,i,j)=[T_drive;delta];
        dU(:,i,j)=U(:,i,j)-input_last;

        ddelta_ori=dU(2,i,j);
        ddelta_abs=min(abs(ddelta_ori),MPC_param.DeltaDeltaMax);
        ddelta=sign(ddelta_ori)*ddelta_abs;
        dT_ori=dU(1,i,j);
        dT_abs=min(abs(dT_ori),MPC_param.DeltaTMax);
        dT_drive=sign(dT_ori)*dT_abs;
        dU(:,i,j)=[dT_drive;ddelta];
        U(:,i,j)=dU(:,i,j)+input_last;
        if i==800
            pause(0.1)
        end
        state_vehicle(:,i+1,j)= VehicleModel_DistributedDrive(state_vehicle(:,i,j),vehicle_param(:,j),U(:,i,j),cphi(ind_ref(i+1,j)),Tstep);
        if j==1
            [history_cost(i,j)] = ILDMPCCostFunction(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),MPC_param,vehicle_param(:,j),state_vehicle(:,i,j),0);
            [nonlinear_constrains(:,i,j)] = ILDMPCConstrain(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),MPC_param,vehicle_param(:,j));
            if state_vehicle_frenet(1,i,j)>=s_withz(end)
                cover_time(j)=i;
                [~,targer_index(j)]=min(abs(state_vehicle_frenet(1,1:i,j)-state_vehicle_frenet(1,1,j)-d_s));
                [state_vehicle_frenet(:,i+1,j),road_condition(:,i+1,j),ind_ref(i+1,j),~] = VehicleLocationOnFrenet(state_vehicle(:,i+1,j),reference_path,Ld,ind_ref(i+1,j));
                break
            end
        else
            if cover_time(j-1)==1
                cover_time(j-1)=T_max;
            end
            
            [history_cost(i,j)] = ILDMPCCostFunction(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),MPC_param,vehicle_param(:,j),state_vehicle(:,i,j),0,state_model_predicted(:,1:end_pre_traj(j),j-1));
            [nonlinear_constrains(:,i,j)] = ILDMPCConstrain(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),MPC_param,vehicle_param(:,j));
            if state_vehicle_frenet(1,i,j)>=state_vehicle_frenet(1,cover_time(j-1),j-1)-d_s
                cover_time(j)=i;
                [~,targer_index(j)]=min(abs(state_vehicle_frenet(1,1:i,j)-state_vehicle_frenet(1,1,j)-d_s));
                [state_vehicle_frenet(:,i+1,j),road_condition(:,i+1,j),ind_ref(i+1,j),~] = VehicleLocationOnFrenet(state_vehicle(:,i+1,j),reference_path,Ld,ind_ref(i+1,j));
                break
            end
        end

        %模型验证
%         if mod(i,100)==0
%             state_model_predicted(:,i+1,j)=ILMPCVehicleModel([state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],vehicle_param(:,j),dU(:,i,j),input_last,road_condition(:,i,j),Tstep);%消除累积误差
%         else
%             state_model_predicted(:,i+1,j)=ILMPCVehicleModel(state_model_predicted(:,i,j),vehicle_param(:,j),dU(:,i,j),input_last,road_condition(:,i,j),Tstep);
%         end
    end      
end
toc
if cover_time(N)==1
    cover_time(N)=T_max;
end

%% 模型验证


%% 画图
figure
% num_row=max(floor(N/2),1);
% num_col=max(ceil(N/num_row),1);
line_type={':','-.','--','-'};
line_width={5,3,2,1};
ding_type={'o','v','d','*'};
plot(reference_path(1,:),reference_path(2,:))
hold on
for k=1:N
    i=cover_time(k);
    plot(state_vehicle(1,1:i,k),state_vehicle(2,1:i,k),LineStyle=line_type{k},LineWidth=line_width{k})
    hold on
%     scatter(state_vehicle(1,i,k),state_vehicle(2,i,k),sqrt(line_width{k}),ding_type{k})
end
legend('reference','vehicle1','vehicle2','vehicle3','vehicle4')
xlabel('global x-coordinate/(m)')
ylabel('global y-coordinate/(m)')
title('traveling trajectory')
xlim(1.2.*[-x_curve-R_path,x_curve+R_path])
ylim(1.5.*[-R_path,R_path])

y_label_stateplot={'station/(m)','distance/(m)','angle/(rad)','velocity/(m/s)','velocity/(m/s)','rate/(rad/s)','acceleration/(m/s^2)','acceleration/(m/s^2)','cost/(-)'};
x_label_stateplot={'time/(s)','time/(s)','time/(s)','time/(s)','time/(s)','time/(s)','time/(s)','time/(s)','time/(s)'};
title_stateplot={'vehicle station','vehicle distance','delta yaw angle','longitudinal velocity','lateral velocity','global yaw rate','longitudinal acceleration','lateral acceleration','recorded step loss'};
figure 
for ind_stateplot=1:9
    subplot(3,3,ind_stateplot)
    for k=1:N
        i=cover_time(k);
        if ind_stateplot<=3
            plot(Tstep.*(1:i),state_vehicle_frenet(ind_stateplot,1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
        elseif ind_stateplot<=8
            plot(Tstep.*(1:i),state_vehicle(ind_stateplot,1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
        else
            plot(Tstep.*(1:i),history_cost(1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
        end
        hold on
    end
    legend('vehicle1','vehicle2','vehicle3','vehicle4')
    ylabel(y_label_stateplot{ind_stateplot})
    xlabel(x_label_stateplot{ind_stateplot})
    title(title_stateplot{ind_stateplot})
end

% figure 
% for ind_stateplot=1:8
%     subplot(3,3,ind_stateplot)
%     for k=1:N
%         i=cover_time(k);
%         plot(Tstep.*(1:i),state_model_predicted(ind_stateplot,1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
%         hold on
%     end
%     legend('vehicle1','vehicle2','vehicle3','vehicle4')
%     ylabel(y_label_stateplot{ind_stateplot})
%     xlabel(x_label_stateplot{ind_stateplot})
%     title(title_stateplot{ind_stateplot})
% end

figure
control_title={'driving torque','wheel steer'};
for ii=1:2
    subplot(2,2,ii)
    for k=1:N
        if cover_time(k)==1
            i=T_max;
        else
            i=cover_time(k);
        end
        plot(Tstep.*(1:i),U(ii,1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
        hold on
    end
    legend('vehicle1','vehicle2','vehicle3','vehicle4')
    title(control_title{ii})
end
hold off

figure
for ii=1:N_nonlinearconstrain
    subplot(3,3,ii)
    for k=1:N
        i=cover_time(k);
        plot(Tstep.*(1:i),nonlinear_constrains(ii,1:i,k),LineStyle=line_type{k},LineWidth=0.5*line_width{k})
        hold on
    end
    legend('vehicle1','vehicle2','vehicle3','vehicle4')
    ylabel('constrain/(-)')
    xlabel('time/(s)')
    title('nonlinear constrain')
end
%% 数据保存
final_database.VehicleStatesXOY=state_vehicle;%[x;y;theta;Vx;Vy;theta_dot;ax;ay];
final_database.VehicleStatesFrenet=state_vehicle_frenet;%[s;t;delta_theta];
final_database.VehicleStatesModel=[state_vehicle_frenet;state_vehicle(4:end,:,:)];%[s;t;delta_theta;Vx;Vy;theta_dot;ax;ay];
final_database.RoadCondition=road_condition;%[phi;curvature]
final_database.ControlInput=U;%[T_drive;delta_steer]
final_database.ControlInputIncrement=dU;%[delta_T_drive;delta_delta_steer]
final_database.RecordedCost=history_cost;%steo loss recorded
final_database.OverTime=cover_time;
final_database.VehicleModelParameter=vehicle_param;
final_database.GlobalReferencePath=reference_path;
final_database.LoadingPathIndex=[loading_start,loading_end];
final_database.LoadingPath=reference_path(:,loading_start:loading_end);
final_database.LoadingTorlerateLength=torlelength;
final_database.MissionTargetVelocity=[transport_v,loading_v];
final_database.MissionMaxVelocity=[transport_v,loading_v]*Gain_LoadSca;
% save('iteration0.mat','final_database');
save('ILMPC_baseline.mat','final_database');