function [total_cost] = ILDMPCCostFunction(dU_sequence,input_last,state_now,road_condition,MPC_param,vehicle_param,history_traj,history_loss,pre_traj)
%% 控制增量序列计算
Nc=MPC_param.ContrHori;%控制域
Nu=MPC_param.ContrLengt;
Nx=MPC_param.StateLengt;
Tstep=MPC_param.SamplTime;%时间步长
model_switch=MPC_param.TermCostAct;
Np=MPC_param.PredHori;%预测域
chosen_num=MPC_param.MinNum;
P=MPC_param.WeiPerfo;
Q=MPC_param.WeiIncre;
R=MPC_param.WeiTrans;
W=MPC_param.WeiSimilJudg;
inf_value=MPC_param.InfRepla;
N_chosen=MPC_param.MinNum;
threshold_judge=MPC_param.ThresSimilJudg;
end_station=MPC_param.EndStati;
pre_s=MPC_param.PreStatiTimeHead;
v_target=MPC_param.TransTargtVel;
v_max=MPC_param.TransMaxVel;
v_min=MPC_param.TransMinVel;
ax_bound=MPC_param.AccLgtMax;
ay_bound=MPC_param.AccLatMax;
vx_punishzone_p=MPC_param.VxBoundTolerPosi;
ax_punishzone_p=MPC_param.AxBoundTolerPosi;
ay_punishzone_p=MPC_param.AyBoundTolerPosi;
vx_punishzone_n=MPC_param.VxBoundTolerNega;
ax_punishzone_n=MPC_param.AxBoundTolerNega;
ay_punishzone_n=MPC_param.AyBoundTolerNega;
ax_max=MPC_param.AccLgtMax;
ay_max=MPC_param.AccLatMax;
ax_min=-ax_max;
ay_min=-ay_max;
t_max=MPC_param.LatDisMax;
t_min=MPC_param.LatDisMin;
t_punishzone_p=MPC_param.LatDistBoundTolerPosi;%车道越界惩罚区间
t_punishzone_n=MPC_param.LatDistBoundTolerNega;%车道越界惩罚区间
scal_fac=MPC_param.PnishScalFactr;

dTq=zeros(1,Np);
ddelta=zeros(1,Np);
for j=1:Np
    if j<=Nc
        dTq(j)=dU_sequence(Nu*j-Nu+1);
        ddelta(j)=dU_sequence(Nu*j-Nu+2);
    else
%         dTq(j)=dU_sequence(Nu*Nc-1);
%         ddelta(j)=dU_sequence(Nu*Nc);
        dTq(j)=0;
        ddelta(j)=0;
    end
end
%% 序列预测
[state_predicted] = ILMPCVehicleModel(state_now,vehicle_param,[dTq;ddelta],input_last,road_condition,Tstep);
state_sequence=[state_now,state_predicted];
state_vector=state_sequence(:);
%% 参考生成
d_s=MPC_param.InterVehDis;
d_h=MPC_param.SafeVehDis;
% d0=d_s+d_h*state_now(4);
d0=d_s;
if nargin<9
    ds_predict_sequence=[0,(tril(ones(Np))*(v_target.*Tstep.*ones(Np,1)))'];
    s_predict_sequence=state_now(1)+ds_predict_sequence;
    s_end_sequence=repmat(end_station,1,Np+1);
    s_target_sequence=min([s_predict_sequence;s_end_sequence]);
    s_target=s_target_sequence;    
    t_target=0.*ones(1,Np+1);
    delta_theta_target=0.*ones(1,Np+1);
    vx_target=v_target+0.*state_sequence(4,:);
    vy_target=0.*ones(1,Np+1);
    omega_target=road_condition(2)/cos(road_condition(1)).*state_sequence(4,:);
    if Np==0
        ax_design=(v_target-state_now(4))/(Tstep);
    else
        ax_design=(v_target-state_now(4))/(Np*Tstep);
    end
    
    ax_design_sequence=repmat(ax_design,1,Np+1);
    ax_max_sequence=repmat(ax_bound,1,Np+1);
    ax_min_sequence=repmat(-ax_bound,1,Np+1);
    ax_max_design=min([ax_design_sequence;ax_max_sequence]);
    ax_min_design=max([ax_max_design;ax_min_sequence]);
    ax_target=ax_min_design;

    if Np==0
        ay_design=(0-state_now(5))/(Tstep);
    else
        ay_design=(0-state_now(5))/(Np*Tstep);
    end
    ay_design_sequence=repmat(ay_design,1,Np+1);
    ay_max_sequence=repmat(ay_bound,1,Np+1);
    ay_min_sequence=repmat(-ay_bound,1,Np+1);
    ay_max_design=min([ay_design_sequence;ay_max_sequence]);
    ay_min_design=max([ay_max_design;ay_min_sequence]);
    ay_target=ay_min_design;
    reference_observe=[s_target;t_target;delta_theta_target;vx_target;vy_target;omega_target;ax_target;ay_target];
else
    total_length=size(pre_traj,2);
    %% 方案1：瞬时协同期望
    ax_target=repmat(pre_traj(7,total_length),1,Np+1);
%     ax_target=(1+Np)/Np*pre_traj(7,total_length)-pre_traj(7,total_length)/Np*(1:Np+1);

    dv_predict_sequence=[0,( tril(ones(Np))*(ax_target(1:Np).*Tstep)' )'];
    vx_target=pre_traj(4,total_length)+dv_predict_sequence;

    ds_predict_sequence=[0,( tril(ones(Np))*(vx_target(1:Np).*Tstep)' )'];
    s_target=pre_traj(1,total_length)-d0+ds_predict_sequence;


%     s_target=repmat(pre_traj(1,total_length)-d0,1,Np+1);
    t_target=zeros(1,Np+1);
    delta_theta_target=0.*ones(1,Np+1);
%     vx_target=repmat(pre_traj(4,total_length),1,Np+1);
    vy_target=zeros(1,Np+1);
    omega_target=road_condition(2)/cos(road_condition(1)).*state_sequence(4,:);
%     ax_target=repmat(pre_traj(7,total_length),1,Np+1);
    if Np==0
        ay_design=(0-state_now(5))/(Tstep);
    else
        ay_design=(0-state_now(5))/(Np*Tstep);
    end
    ay_design_sequence=repmat(ay_design,1,Np+1);
    ay_max_sequence=repmat(ay_bound,1,Np+1);
    ay_min_sequence=repmat(-ay_bound,1,Np+1);
    ay_max_design=min([ay_design_sequence;ay_max_sequence]);
    ay_min_design=max([ay_max_design;ay_min_sequence]);
    ay_target=ay_min_design;
    %% 方案2：路况期望
%     [~,min_index]=min(abs(pre_traj(1,:)-pre_traj(1,total_length)+d0));
%     %假设初始轨迹满足要求
%     if min_index+Np>total_length
%         pre_traj=[pre_traj,repmat(pre_traj(:,total_length),1,min_index+Np+1-total_length)];
%     end
%     s_target=pre_traj(1,min_index:min_index+Np);
%     t_target=0.*ones(1,Np+1);
%     delta_theta_target=0.*ones(1,Np+1);
%     vx_target=pre_traj(4,min_index:min_index+Np);
%     vy_target=0.*ones(1,Np+1);
%     omega_target=road_condition(2)/cos(road_condition(1)).*state_sequence(4,:);
%     ax_target=pre_traj(7,min_index:min_index+Np);
%     ay_target=pre_traj(8,min_index:min_index+Np);
    reference_observe=[s_target;t_target;delta_theta_target;vx_target;vy_target;omega_target;ax_target;ay_target];
%     scatter(state_now(1),ax_target(1))
%     pause(0.1)
%     hold on
end
reference_vector=reference_observe(:);

N_reference=length(reference_vector)/(Np+1);
%% 代价函数

C=[eye(N_reference),zeros(N_reference,Nx-N_reference)];
observe_vector=kron(eye(Np+1),C)*state_vector;
loss_performance=(observe_vector-reference_vector)'*kron(eye(Np+1),P(1:N_reference,1:N_reference))*(observe_vector-reference_vector);%控制表现损失
loss_increment=dU_sequence'*kron(eye(Nc),Q)*dU_sequence;%控制增量损失
velocity_vector=observe_vector(4:N_reference:end);
velocity_bounded_p=(v_max-vx_punishzone_p-velocity_vector);%vx_punishzone_p>0
velocity_bounded_error_flag_p=velocity_bounded_p<0;
velocity_bounded_error_p=velocity_bounded_p.*velocity_bounded_error_flag_p;
%车速正越界惩罚
punishment_velocity_p=scal_fac*velocity_bounded_error_p'*kron(eye(Np+1),P(4,4))*velocity_bounded_error_p;
velocity_bounded_n=(velocity_vector-v_min-vx_punishzone_n);%vx_punishzone_n>0
velocity_bounded_error_flag_n=velocity_bounded_n<0;
velocity_bounded_error_n=velocity_bounded_n.*velocity_bounded_error_flag_n;
%车速负越界惩罚
punishment_velocity_n=scal_fac*velocity_bounded_error_n'*kron(eye(Np+1),P(4,4))*velocity_bounded_error_n;
ax_vector=observe_vector(7:N_reference:end);
ax_bounded_p=(ax_max-ax_punishzone_p-ax_vector);%ax_punishzone_p>0
ax_bounded_error_flag_p=ax_bounded_p<0;
ax_bounded_error_p=ax_bounded_p.*ax_bounded_error_flag_p;
punishment_ax_p=scal_fac*ax_bounded_error_p'*kron(eye(Np+1),P(7,7))*ax_bounded_error_p;%加速正越界惩罚
ax_bounded_n=(ax_vector-ax_min+ax_punishzone_n);%ax_punishzone_n<0
ax_bounded_error_flag_n=ax_bounded_n<0;
ax_bounded_error_n=ax_bounded_n.*ax_bounded_error_flag_n;
punishment_ax_n=scal_fac*ax_bounded_error_n'*kron(eye(Np+1),P(7,7))*ax_bounded_error_n;%加速负越界惩罚
ay_vector=observe_vector(8:N_reference:end);
ay_bounded_p=(ay_max-ay_punishzone_p-ay_vector);%ay_punishzone_p>0
ay_bounded_error_flag_p=ay_bounded_p<0;
ay_bounded_error_p=ay_bounded_p.*ay_bounded_error_flag_p;
punishment_ay_p=scal_fac*ay_bounded_error_p'*kron(eye(Np+1),P(8,8))*ay_bounded_error_p;%加速正越界惩罚
ay_bounded_n=(ay_vector-ay_min+ay_punishzone_n);%ay_punishzone_n<0
ay_bounded_error_flag_n=ay_bounded_n<0;
ay_bounded_error_n=ay_bounded_n.*ay_bounded_error_flag_n;
punishment_ay_n=scal_fac*ay_bounded_error_n'*kron(eye(Np+1),P(8,8))*ay_bounded_error_n;%加速负越界惩罚
t_vector=observe_vector(2:N_reference:end);
t_bounded_p=(t_max-t_punishzone_p-t_vector);%t_punishzone_p>0
t_bounded_error_flag_p=t_bounded_p<0;
t_bounded_error_p=t_bounded_p.*t_bounded_error_flag_p;
punishment_t_p=scal_fac*t_bounded_error_p'*kron(eye(Np+1),P(2,2))*t_bounded_error_p;%车道正越界惩罚
t_bounded_n=(t_vector-t_min+t_punishzone_n);%t_punishzone_n<0
t_bounded_error_flag_n=t_bounded_n<0;
t_bounded_error_n=t_bounded_n.*t_bounded_error_flag_n;
punishment_t_n=scal_fac*t_bounded_error_n'*kron(eye(Np+1),P(2,2))*t_bounded_error_n;%车道负越界惩罚
punishment_total=punishment_ay_n+punishment_ay_p+punishment_ax_n+punishment_ax_p+punishment_velocity_n+punishment_velocity_p+punishment_t_p+punishment_t_n;
s_vector=observe_vector(1:N_reference:end);
% loss_transport=(s_vector-end_station)'*R*(s_vector-end_station);%快速运输损失
loss_transport=0;

terminal_state=state_vector(end-Nx+1:end);
history_vector=history_traj(:);
history_num=length(history_vector)/Nx;
relative_difference=zeros(history_num,1);
for i=1:history_num
    relative_difference(i)=(history_traj(:,i)-terminal_state)'*W*(history_traj(:,i)-terminal_state);
end
[value_chosen,index_chosen]=mink(relative_difference,chosen_num);%取状态间距离最小的k个元素
if value_chosen(1)>=threshold_judge
    terminal_error=terminal_state-history_traj(:,index_chosen(1));
    loss_terminal= inf_value+terminal_error'*P*terminal_error;%迭代学习终端损失
else
    reached_points_num=sum(value_chosen<threshold_judge);
    loss_terminal= min( history_loss(index_chosen(1:reached_points_num)) );
end
if model_switch==1
    total_cost=loss_performance+loss_increment+loss_transport+loss_terminal+punishment_total;
else
    total_cost=loss_performance+loss_increment+loss_transport+punishment_total;
end
end