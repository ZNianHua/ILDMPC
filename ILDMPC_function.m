function ILDMPC_function(load_iteration_ind)
%% 参考路径
% load_iteration_ind=0;
load(['iteration',num2str(load_iteration_ind),'.mat'])
reference_path=final_database.GlobalReferencePath;
loading_path=final_database.LoadingPath;
loading_start=final_database.LoadingPathIndex(1);
loading_end=final_database.LoadingPathIndex(2);
torlelength=final_database.LoadingTorlerateLength;

transport_v_bound=final_database.MissionMaxVelocity(1);

loading_v_bound=final_database.MissionMaxVelocity(2);
if load_iteration_ind==0
    transport_v=24;
    loading_v=24;
else
    transport_v=final_database.MissionTargetVelocity(1);
    loading_v=final_database.MissionTargetVelocity(2);
end
history_traj=final_database.VehicleStatesModel;
history_cost=final_database.RecordedCost;
state_recorded_XOY=final_database.VehicleStatesXOY;
end_time=final_database.OverTime;
vehicle_param=final_database.VehicleModelParameter;
ds=mean(reference_path(end,2:end)-reference_path(end,1:end-1));
%% 仿真初始化
T_max=30000;Tstep=0.01;
Nx=8;Nx_frenet=3;Nx_road=2;N_nonlinearconstrain=8;Nu=2;
d_s=10;%恒间距策略距离
N=4;%编队数量
targer_index=ones(1,N);
optimize_flag=zeros(T_max,N);
optimal_value=zeros(T_max,N);
recorded_cost=zeros(T_max,N);
ind_ref=ones(1,N);
cover_time=ones(1,N);
state_vehicle=zeros(Nx,T_max+1,N);
state_vehicle_frenet=zeros(Nx_frenet,T_max,N);
state_model_predicted=zeros(Nx,T_max,N);
road_condition=zeros(Nx_road,T_max,N);
rand_state=[1;1;0.1;1;1;0.1;1].*0.1;
if length(rand_state)~=Nx
    rand_state=[rand_state;zeros(Nx-length(rand_state),1)];
end

for ind_intial=1:N
    state_vehicle(:,1,ind_intial)=(2*rand(1)-1)*rand_state+state_recorded_XOY(:,1,ind_intial);
    ind_ref(ind_intial)=max(1,round( (state_vehicle(1,1,ind_intial)/ds) ));
end
U=zeros(Nu,T_max,N);
dU=zeros(Nu,T_max,N);
phi=zeros(T_max,N);

%% MPC参数
No=2;Np=30;Nc=5;h=0.3;vel_acc_torler_percent=0.2;
options=optimset('Algorithm','active-set');
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
MPC_param.EndStati=reference_path(end,end);%终点s坐标
MPC_param.MinNum=5;%计算终端代价时，判断当前状态与历史状态集中点重合过程所取的前n个历史状态
MPC_param.ThresSimilJudg=1;%计算终端代价时，判断当前状态与历史状态集中点重合过程相对差异阈值
MPC_param.InfRepla=1e10;%计算终端代价时，无穷大的替代值

MPC_param.AccLgtMax=8;
MPC_param.AccLatMax=5;
MPC_param.LatDisMax=3;
MPC_param.LatDisMin=-3;
MPC_param.TermCostAct=1;%终端代价启用使能
MPC_param.PreStatiTimeHead=0.1;%为领航车准备的前向预瞄时距
MPC_param.PnishScalFactr=20;
MPC_param.AxBoundTolerPosi=vel_acc_torler_percent*MPC_param.AccLgtMax;%纵向加速度越界惩罚区间
MPC_param.AyBoundTolerPosi=vel_acc_torler_percent*MPC_param.AccLatMax;%横向加速度越界惩罚区间
MPC_param.AxBoundTolerNega=-MPC_param.AxBoundTolerPosi;%纵向加速度越界惩罚区间
MPC_param.AyBoundTolerNega=-MPC_param.AyBoundTolerPosi;%横向加速度越界惩罚区间
MPC_param.LatDistBoundTolerPosi=vel_acc_torler_percent*MPC_param.LatDisMax;%纵向加速度越界惩罚区间
MPC_param.LatDistBoundTolerNega=vel_acc_torler_percent*MPC_param.LatDisMin;%横向加速度越界惩罚区间



% nonlinear_constrains=zeros(N_nonlinearconstrain,T_max,N);
end_pre_traj=ones(1,N);
%% 仿真过程
% Gain_delta=[0,0,-0.02,-0.3];
% Gain_drive=[50,110,0,0];
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
        ub_constrain=repmat([ub1;ub2],Nc,1);
        lb_constrain=repmat([lb1;lb2],Nc,1);
        %% XOY2Frenet
        [state_vehicle_frenet(:,i,j),road_condition(:,i,j),ind_ref(j),~] = VehicleLocationOnFrenet(state_vehicle(:,i,j),reference_path,0,ind_ref(j));
        if state_vehicle_frenet(1,i,j)>(loading_path(end,1)-torlelength) && state_vehicle_frenet(1,i,j)<(loading_path(end,end)+torlelength)
            MPC_param.TransTargtVel=loading_v;
        else
            MPC_param.TransTargtVel=transport_v;
        end
        if state_vehicle_frenet(1,i,j)>(loading_path(end,1)) && state_vehicle_frenet(1,i,j)<(loading_path(end,end))
            MPC_param.TransMaxVel=loading_v_bound;
            MPC_param.VxBoundTolerPosi=vel_acc_torler_percent*MPC_param.TransMaxVel;%纵向速度越界惩罚区间
        else
            MPC_param.TransMaxVel=transport_v_bound;
            MPC_param.VxBoundTolerPosi=vel_acc_torler_percent*MPC_param.TransMaxVel;%纵向速度越界惩罚区间
        end
        MPC_param.TransMinVel=0;
        MPC_param.VxBoundTolerNega=MPC_param.VxBoundTolerPosi;

        %线性约束
        if i==1
            input_last=[0;0];
        else
            input_last=U(:,i-1,j);
        end
        state_model_predicted(:,i,j)=[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)];
        u_UB_vector=repmat([Tq_max;delta_max],Nc,1);
        u_LB_vector=repmat([Tq_min;delta_min],Nc,1);
        b1=u_UB_vector-repmat(input_last,Nc,1);
        A1=kron(tril(ones(Nc)),eye(Nu));
        b2=repmat(input_last,Nc,1)-u_LB_vector;
        A2=-A1;
        b_constrain=[b1;b2];
        A_constrain=[A1;A2];

        if j==1
%             velocity_error=MPC_param.TransTargtVel-state_vehicle(4,i,j);
%             station_error=0;
%             error_state=[station_error;velocity_error;state_vehicle_frenet(2:3,i,j)];
%             delta_ori=Gain_delta*error_state;
%             delta_abs=min(abs(delta_ori),delta_max);
%             delta=sign(delta_ori)*delta_abs;
%             T_ori=Gain_drive*error_state;
%             T_abs=min(abs(T_ori),Tq_max);
%             T_drive=sign(T_ori)*T_abs;
%             U_feedback=[T_drive;delta];
%             dU_feedback=U_feedback-input_last;
%             ddelta_ori=dU_feedback(2);
%             ddelta_abs=min(abs(ddelta_ori),MPC_param.DeltaDeltaMax);
%             ddelta=sign(ddelta_ori)*ddelta_abs;
%             dT_ori=dU_feedback(1);
%             dT_abs=min(abs(dT_ori),MPC_param.DeltaTMax);
%             dT_drive=sign(dT_ori)*dT_abs;
%             u_best=[dT_drive;ddelta];
            [u_best,optimal_value(i,j),optimize_flag(i,j)]=fmincon(@(dU_sequence) ILDMPCCostFunction(dU_sequence,input_last,state_model_predicted(:,i,j),road_condition(:,i,j),MPC_param,vehicle_param(:,j),history_traj(:,1:end_time(j),j),history_cost(1:end_time(j),j)),zeros(Nu*Nc,1),A_constrain,b_constrain,[],[],lb_constrain,ub_constrain,@(dU_sequence) ILDMPCConstrain(dU_sequence,input_last,state_model_predicted(:,i,j),road_condition(:,i,j),MPC_param,vehicle_param(:,j)),options);
        else
            if cover_time(j-1)==1
                end_time_pre=T_max;
            else
                end_time_pre=cover_time(j-1);
            end
            
%             end_pre_traj(j)=max([targer_index(j-1),min(i,cover_time(j-1)),end_pre_traj(j)+1]);
            end_pre_traj(j)=min(i,cover_time(j-1));
            [u_best,optimal_value(i,j),optimize_flag(i,j)]=fmincon(@(dU_sequence) ILDMPCCostFunction(dU_sequence,input_last,state_model_predicted(:,i,j),road_condition(:,i,j),MPC_param,vehicle_param(:,j),history_traj(:,1:end_time(j),j),history_cost(1:end_time(j),j),state_model_predicted(:,1:end_pre_traj(j),j-1)),zeros(Nu*Nc,1),A_constrain,b_constrain,[],[],lb_constrain,ub_constrain,@(dU_sequence) ILDMPCConstrain(dU_sequence,input_last,state_model_predicted(:,i,j),road_condition(:,i,j),MPC_param,vehicle_param(:,j)),options);
        end
        dU(:,i,j)=u_best(1:2);
        if optimize_flag(i,j)==-2
            fprintf(['ERROR: vehicle',num2str(j),' has constrains that can not be satisfied']);
            break
        end
%         if i==180
%             pause(0.1)
%         end
        U(:,i,j)=dU(:,i,j)+input_last;
        state_vehicle(:,i+1,j)= VehicleModel_DistributedDrive(state_vehicle(:,i,j),vehicle_param(:,j),U(:,i,j),road_condition(1,i,j),Tstep);
        recorded_param=MPC_param;
        recorded_param.TermCostAct=0;
        recorded_param.PredHori=1;
        recorded_param.ContrHori=1;
        if j==1
            [recorded_cost(i,j)] = ILDMPCCostFunction(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),recorded_param,vehicle_param(:,j),state_vehicle(:,i,j),0);
            if state_vehicle_frenet(1,i,j)>=reference_path(end,end)
                cover_time(j)=i;
                [~,targer_index(j)]=min(abs(state_vehicle_frenet(1,1:i,j)-state_vehicle_frenet(1,1,j)-d_s));
                [state_vehicle_frenet(:,i+1,j),road_condition(:,i+1,j),ind_ref(j),~] = VehicleLocationOnFrenet(state_vehicle(:,i+1,j),reference_path,0,ind_ref(j));
                break
            end
        else
            [recorded_cost(i,j)] = ILDMPCCostFunction(dU(:,i,j),input_last,[state_vehicle_frenet(1:3,i,j);state_vehicle(4:end,i,j)],road_condition(:,i,j),recorded_param,vehicle_param(:,j),state_vehicle(:,i,j),0,state_model_predicted(:,1:end_pre_traj(j),j-1));
            if state_vehicle_frenet(1,i,j)>=state_vehicle_frenet(1,end_time_pre,j-1)-d_s
                cover_time(j)=i;
                [~,targer_index(j)]=min(abs(state_vehicle_frenet(1,1:i,j)-state_vehicle_frenet(1,1,j)-d_s));
                [state_vehicle_frenet(:,i+1,j),road_condition(:,i+1,j),ind_ref(j),~] = VehicleLocationOnFrenet(state_vehicle(:,i+1,j),reference_path,0,ind_ref(j));
                break
            end
        end

        %% 实时展示跟踪效果
%         if mod(i,10)==1
%             scatter(reference_path(1,ind_ref(j)),reference_path(2,ind_ref(j)),'+')
%             hold on
%             scatter(state_vehicle(1,i+1,j),state_vehicle(2,i+1,j))
%             pause(0.1)
%         end
    end      
end
toc
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
    if cover_time(k)==1
        i=T_max;
    else
        i=cover_time(k);
    end
    plot(state_vehicle(1,1:i,k),state_vehicle(2,1:i,k),LineStyle=line_type{k},LineWidth=line_width{k})
    hold on
%     scatter(state_vehicle(1,i,k),state_vehicle(2,i,k),sqrt(line_width{k}),ding_type{k})
end
legend('reference','vehicle1','vehicle2','vehicle3','vehicle4')
xlabel('global x-coordinate/(m)')
ylabel('global y-coordinate/(m)')
title('traveling trajectory')


%% 数据保存
final_database.VehicleStatesXOY=state_vehicle;%[x;y;theta;Vx;Vy;theta_dot;ax;ay];
final_database.VehicleStatesFrenet=state_vehicle_frenet;%[s;t;delta_theta];
final_database.VehicleStatesModel=[state_vehicle_frenet;state_vehicle(4:end,1:end-1,:)];%[s;t;delta_theta;Vx;Vy;theta_dot;ax;ay];
final_database.RoadCondition=road_condition;%[phi;curvature]
final_database.ControlInput=U;%[T_drive;delta_steer]
final_database.ControlInputIncrement=dU;%[delta_T_drive;delta_delta_steer]
final_database.RecordedCost=recorded_cost;%step loss recorded
final_database.OverTime=cover_time;
final_database.VehicleModelParameter=vehicle_param;
final_database.GlobalReferencePath=reference_path;
final_database.LoadingPathIndex=[loading_start,loading_end];
final_database.LoadingPath=reference_path(:,loading_start:loading_end);
final_database.LoadingTorlerateLength=torlelength;
final_database.MissionTargetVelocity=[transport_v,loading_v];
final_database.MissionMaxVelocity=[transport_v_bound,loading_v_bound];
save_iteration=['iteration',num2str(load_iteration_ind+1),'.mat'];
save(save_iteration,'final_database');
end