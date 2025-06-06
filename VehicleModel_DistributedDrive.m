function [state_next,Power_vector] = VehicleModel_DistributedDrive(state_pre,vehicle_model_param,U,phi,Tstep,uncertain_flag)
Nx=8;
if nargin<6
    uncertain_flag=zeros(Nx,1);
end
%% 除零阈值
vx_thres=0.001;
vy_thres=0.001;
theta_dot_thres=1e-5;

M=vehicle_model_param(1);
Iz=vehicle_model_param(2);
a=vehicle_model_param(3);
b=vehicle_model_param(4);
tfl=vehicle_model_param(5);
tfr=vehicle_model_param(6);
trl=vehicle_model_param(7);
trr=vehicle_model_param(8);
hg=vehicle_model_param(9);
Ca=vehicle_model_param(10);
Cfl=vehicle_model_param(11);
Cfr=vehicle_model_param(12);
Crl=vehicle_model_param(13);
Crr=vehicle_model_param(14);
miufl=vehicle_model_param(15);
miufr=vehicle_model_param(16);
miurl=vehicle_model_param(17);
miurr=vehicle_model_param(18);
taux=vehicle_model_param(19);
tauy=vehicle_model_param(20);
Rfl=vehicle_model_param(21);
Rfr=vehicle_model_param(22);
Rrl=vehicle_model_param(23);
Rrr=vehicle_model_param(24);
ifl=vehicle_model_param(25);
ifr=vehicle_model_param(26);
irl=vehicle_model_param(27);
irr=vehicle_model_param(28);
eta=vehicle_model_param(29);
g=vehicle_model_param(30);

L=a+b;
Bf=tfr+tfl;
Br=trr+trl;

if length(U)==6
    T_fl=U(1);
    T_fr=U(2);
    T_rl=U(3);
    T_rr=U(4);
    delta_l=U(5);
    delta_r=U(6);
else
    T_fl=U(1)/4;
    T_fr=U(1)/4;
    T_rl=U(1)/4;
    T_rr=U(1)/4;
    delta_l=U(2);
    delta_r=U(2);   
end

Xp=state_pre(1);
Yp=state_pre(2);
thetap=state_pre(3);
Vxp=state_pre(4);
Vyp=state_pre(5);
theta_dotp=state_pre(6);
axp=state_pre(7);
ayp=state_pre(8);

Fzfl=(M*g*b*cos(phi)/L-M*g*hg*sin(phi)/L-M*hg*axp/L)*(tfr/Bf)+hg*M*ayp/Bf;
Fzfr=(M*g*b*cos(phi)/L-M*g*hg*sin(phi)/L-M*hg*axp/L)*(tfl/Bf)-hg*M*ayp/Bf;
Fzrl=(M*g*a*cos(phi)/L+M*g*hg*sin(phi)/L+M*hg*axp/L)*(trr/Br)+hg*M*ayp/Br;
Fzrr=(M*g*a*cos(phi)/L+M*g*hg*sin(phi)/L+M*hg*axp/L)*(trl/Br)-hg*M*ayp/Br;

Fxfl=T_fl*eta*ifl/Rfl-miufl*Fzfl;
Fxfr=T_fr*eta*ifr/Rfr-miufr*Fzfr;
Fxrl=T_rl*eta*irl/Rrl-miurl*Fzrl;
Fxrr=T_rr*eta*irr/Rrr-miurr*Fzrr;
F_roll=miufl*Fzfl+miufr*Fzfr+miurl*Fzrl+miurr*Fzrr;


if abs(Vxp)<vx_thres
    alpha_fl=0;
    alpha_fr=0;
    alpha_rl=0;
    alpha_rr=0;
else
    alpha_fl=-(delta_l-atan2((Vyp+a*theta_dotp),(Vxp-tfl*theta_dotp)) );
    alpha_fr=-(delta_r-atan2((Vyp+a*theta_dotp),(Vxp+tfr*theta_dotp)));
    alpha_rl=(atan2((Vyp-b*theta_dotp),(Vxp-trl*theta_dotp)));
    alpha_rr=(atan2((Vyp-b*theta_dotp),(Vxp+trr*theta_dotp)));
end



Fyfl=Cfl*alpha_fl;
Fyfr=Cfr*alpha_fr;
Fyrl=Crl*alpha_rl;
Fyrr=Crr*alpha_rr;

F_air=Ca*abs(Vxp)*Vxp;
F_gradient=M*g*sin(phi);
Fx=Fxfl*cos(delta_l)+Fxfr*cos(delta_r)-Fyfl*sin(delta_l)-Fyfr*sin(delta_r)+Fxrl+Fxrr-M*g*sin(phi)-F_gradient-F_air;
Fy=Fxfl*sin(delta_l)+Fxfr*sin(delta_r)+Fyfl*cos(delta_l)+Fyfr*cos(delta_r)+Fyrl+Fyrr;
Mx=-tfl*Fxfl*cos(delta_l)+tfr*Fxfr*cos(delta_r)+tfl*Fyfl*sin(delta_l)-tfr*Fyfr*sin(delta_r)-trl*Fxrl+trr*Fxrr;
My=a*Fxfl*sin(delta_l)+a*Fxfr*sin(delta_r)+a*Fyfl*cos(delta_l)+a*Fyfr*cos(delta_r)-b*Fyrl-b*Fyrr;



Vx_dot=Fx/M+Vyp*theta_dotp;
Vy_dot=Fy/M-Vxp*theta_dotp;
theta_dot_dot=(Mx+My)/Iz;
ax_dot=(Vx_dot-axp)/(taux);
ay_dot=(Vy_dot-ayp)/(tauy);

X=Xp+Tstep*Vxp*cos(thetap)-Tstep*Vyp*sin(thetap)+uncertain_flag(1)*(rand(1)-0.5)*1;
Y=Yp+Tstep*Vxp*sin(thetap)+Tstep*Vyp*cos(thetap)+uncertain_flag(2)*(rand(1)-0.5)*1;
theta=thetap+Tstep*theta_dotp+uncertain_flag(3)*rand(1)*0.1;
Vx=Vxp+Tstep*axp+uncertain_flag(4)*(rand(1)-0.5)*0.5;
Vy=Vyp+Tstep*ayp+uncertain_flag(5)*(rand(1)-0.5)*0.1;
theta_dot=theta_dotp+Tstep*theta_dot_dot+uncertain_flag(6)*(rand(1)-0.5)*0.1;
ax=axp+ax_dot*Tstep+uncertain_flag(7)*(rand(1)-0.5)*0.1;
ay=ayp+ay_dot*Tstep+uncertain_flag(7)*(rand(1)-0.5)*0.1;

P_x=M*axp*Vxp;
P_y=M*ayp*Vyp;
P_roll=F_roll*Vxp;
P_air=F_air*Vxp;

%% 消除累积误差
if abs(Vx)<vx_thres
    Vx=0;
end
if abs(Vy)<vy_thres
    Vy=0;
end
if abs(theta_dot)<theta_dot_thres
    theta_dot=0;
end


state_next=[X;Y;theta;Vx;Vy;theta_dot;ax;ay];
Power_vector=[P_x,P_y,P_roll,P_air]';



end

