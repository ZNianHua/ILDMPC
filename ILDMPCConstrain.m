function [nonlinear_constrains,nonlinear_equation] = ILDMPCConstrain(dU_sequence,input_last,state_now,road_condition,MPC_param,vehicle_param)
%% 控制增量序列计算
Np=MPC_param.PredHori;%预测域
Nc=MPC_param.ContrHori;%控制域
Nu=MPC_param.ContrLengt;
Nx=MPC_param.StateLengt;
Tstep=MPC_param.SamplTime;%时间步长
dTq=zeros(1,Np);
ddelta=zeros(1,Np);
for j=1:Np
    if j<=Nc
        dTq(j)=dU_sequence(Nu*j-Nu+1);
        ddelta(j)=dU_sequence(Nu*j-Nu+2);
    else
        dTq(j)=dU_sequence(Nu*Nc-1);
        ddelta(j)=dU_sequence(Nu*Nc);
    end
end
%% 序列预测
[state_predicted] = ILMPCVehicleModel(state_now,vehicle_param,[dTq;ddelta],input_last,road_condition,Tstep);
state_sequence=[state_now,state_predicted];
%% 约束
% T_max=MPC_param.TMax;%最大扭矩
% delta_max=MPC_param.DeltaMax;%最大转角
% ub1=MPC_param.DeltaTMax;%最大扭矩增量
% ub2=MPC_param.DeltaDeltaMax;%最大转角增量
% T_min=MPC_param.TMin;%最小扭矩
% delta_min=MPC_param.DeltaMin;%最小转角
% lb1=MPC_param.DeltaTMin;%最小扭矩增量
% lb2=MPC_param.DeltaDeltaMin;%最小转角增量
ax_bound=MPC_param.AccLgtMax;
ay_bound=MPC_param.AccLatMax;
t_lb=MPC_param.LatDisMin;
t_ub=MPC_param.LatDisMax;
vx_ub=MPC_param.TransMaxVel;
vx_lb=MPC_param.TransMinVel;
%% 上下界约束
% UB_vector=repmat([ub1;ub2],Nc,1);
% LB_vector=repmat([lb1;lb2],Nc,1);
%% 线性约束
% u_UB_vector=repmat([T_max;delta_max],Nc,1);
% u_LB_vector=repmat([T_min;delta_min],Nc,1);
% b1=u_UB_vector-repmat(input_last,Nc,1);
% A1=kron(tril(ones(Nc)),ones(Nu,1));
% b2=repmat(input_last,Nc,1)-u_LB_vector;
% A2=-A1;
% b=[b1;b2];
% A=[A1;A2];
%% 非线性约束
nc1=state_sequence(2,:)'-t_ub;
nc2=t_lb-state_sequence(2,:)';
nc3=state_sequence(4,:)'-vx_ub;
nc4=vx_lb-state_sequence(4,:)';
nc5=state_sequence(Nx-1,:)'-ax_bound;
nc6=-ax_bound-state_sequence(Nx-1,:)';
nc7=state_sequence(Nx,:)'-ay_bound;
nc8=-ay_bound-state_sequence(Nx,:)';
if Np==1
    nonlinear_constrains=[nc1(1);nc2(1);nc3(1);nc4(1);nc5(1);nc6(1);nc7(1);nc8(1)];
else
    nonlinear_constrains=[nc1;nc2;nc3;nc4;nc5;nc6;nc7;nc8];
end
nonlinear_equation=[];
% nonlinear_constrains=nonlinear_constrains';
end