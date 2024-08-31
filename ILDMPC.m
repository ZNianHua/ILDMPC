function [outputArg1,outputArg2] = ILDMPC(dU_sequence,input_last,state_now,road_condition,MPC_param,vehicle_param,pre_traj)
Np=MPC_param.PredHori;%预测域
Nc=MPC_param.ContrHori;%控制域
Nu=MPC_param.ContrLengt;
Nx=MPC_param.StateLengt;
dTq=zeros(1,Np);
ddelta=zeros(1,Np);
for j=1:Np
    if j<=Nc
        dTq(j)=dU_sequence(Nu*j-Nu+1);
        ddelta(j)=dU_sequence(Nu*j-Nu+2);
    else
        dTq(j)=dU_sequence(Nu*Nc-2);
        ddelta(j)=dU_sequence(Nu*Nc-1);
    end

end
%% 序列预测
[state_sequence] = ILMPCVehicleModel(state_now,vehicle_param,[dTq;ddelta],input_last,road_condition,Tstep);
state_vector=state_sequence(:);
%% 参考生成
d_s=MPC_param.InterVehDis;
d_h=MPC_param.SafeVehDis;
d0=d_s+d_h*state_now(4);
if nargin<7
    s_target=state_sequence(1,:);
    t_target=0;
    delta_theta_target=0;
    vx_target=MPC_param.TarVel;
    vy_target=0;
    omega_target=road_condition(2)*state_sequence(4,:);
    reference_observe=[s_target;t_target.*ones(1,Np);delta_theta_target.*ones(1,Np);vx_target.*ones(1,Np);vy_target.*ones(1,Np);omega_target];
else
    [~,min_index]=min(abs(pre_traj(1,:)-state_now(1)-d0));
    s_target=pre_traj(1,min_index+1:min_index+Np);
    t_target=0;
    delta_theta_target=0;
    vx_target=pre_traj(4,min_index+1:min_index+Np);
    vy_target=0;
    omega_target=road_condition(2)*vx_target;
    reference_observe=[s_target;t_target.*ones(1,Np);delta_theta_target.*ones(1,Np);vx_target;vy_target.*ones(1,Np);omega_target];
end
reference_vetor=reference_observe(:);
%% 代价函数
[total_cost] = ILDMPCCostFunction(increment_vector,state_vector,reference_vector,MPC_param,history_traj,history_loss);
%% 约束
C_acc=[zeros(2,Nx-2),eye(2)];
acc_vector=kron(eye(Np),C_acc)*state_vector;
end