function [state_sequence] = ILMPCVehicleModel(state_now,vehicle_model_param,dU_sequence,U_last,road_condition,Tstep)

%% 除零阈值
vx_thres=0.001;
vy_thres=0.001;
theta_dot_thres=1e-5;

Nx=8;Nu=2;
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
Np=size(dU_sequence,2);
input_sequence=zeros(Nu,Np);
state_sequence=zeros(Nx,Np);
for i=1:Np
    if i==1
        U=U_last+dU_sequence(:,i);
        input_sequence(:,i)=U;
    else
        U=input_sequence(:,i-1)+dU_sequence(:,i);
        input_sequence(:,i)=U;
    end
    
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
    if i==1
        state_current=state_now;
    else
        state_current=state_sequence(:,i-1);
    end
    sp=state_current(1);
    tp=state_current(2);
    delta_thetap=state_current(3);
    Vxp=state_current(4);
    Vyp=state_current(5);
    theta_dotp=state_current(6);
    axp=state_current(7);
    ayp=state_current(8);
    phi=road_condition(1);
%     if road_condition(2)==0
%         curvarure=0;
%         fix_term=0;
%     else
%         curvarure=1/(tp+1/road_condition(2))/cos(phi);
%         fix_term=-0.0171218717816308+0.000442616463254272*Vxp*cos(phi)+0.00667058664084804*tp;
%     end

    if road_condition(2)==0
        curvarure=0;
        fix_term=0;
    else
        curvarure=1/(-tp+1/road_condition(2))/cos(phi);
%         curvarure=road_condition(2)/cos(phi);
        fix_term=0;
    end
    
    Fzfl=(M*g*b*cos(phi)/L-M*g*hg*sin(phi)/L-M*hg*axp/L)*(tfr/Bf)+hg*M*ayp/Bf;
    Fzfr=(M*g*b*cos(phi)/L-M*g*hg*sin(phi)/L-M*hg*axp/L)*(tfl/Bf)-hg*M*ayp/Bf;
    Fzrl=(M*g*a*cos(phi)/L+M*g*hg*sin(phi)/L+M*hg*axp/L)*(trr/Br)+hg*M*ayp/Br;
    Fzrr=(M*g*a*cos(phi)/L+M*g*hg*sin(phi)/L+M*hg*axp/L)*(trl/Br)-hg*M*ayp/Br;
    
    Fxfl=T_fl*eta*ifl/Rfl-miufl*Fzfl;
    Fxfr=T_fr*eta*ifr/Rfr-miufr*Fzfr;
    Fxrl=T_rl*eta*irl/Rrl-miurl*Fzrl;
    Fxrr=T_rr*eta*irr/Rrr-miurr*Fzrr;
    
    
    if abs(Vxp)<vx_thres
        alpha_fl=0;
        alpha_fr=0;
        alpha_rl=0;
        alpha_rr=0;
    else
        alpha_fl=-(delta_l-atan2((Vyp+a*theta_dotp),(Vxp-tfl*theta_dotp)) );
        alpha_fr=-(delta_r-atan2((Vyp+a*theta_dotp),(Vxp+tfr*theta_dotp)));
        alpha_rl=(atan2((Vyp-b*theta_dotp),(Vxp+trl*theta_dotp)));
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
    
    s_dot=Vxp*cos(delta_thetap)-Vyp*sin(delta_thetap);
%     s_dot=Vxp;
    s=sp+Tstep*s_dot;
    t=tp+Tstep*Vxp*sin(delta_thetap)+Tstep*Vyp*cos(delta_thetap);
    delta_theta_dot=theta_dotp-s_dot*curvarure;
    delta_theta=delta_thetap+delta_theta_dot*Tstep-fix_term*Tstep;
    Vx=Vxp+Tstep*axp;
    Vy=Vyp+Tstep*ayp;
    delta_theta_second_dot_road=axp*cos(delta_thetap)-Vxp*delta_theta_dot*sin(delta_thetap)-ayp*sin(delta_thetap)-Vyp*delta_theta_dot*cos(delta_thetap);
%     delta_theta_second_dot_road=axp;
%     theta_dot=theta_dotp+theta_dot_dot*Tstep-delta_theta_second_dot_road*curvarure*Tstep;
    theta_dot=theta_dotp+theta_dot_dot*Tstep;
    ax=axp+ax_dot*Tstep;
    ay=ayp+ay_dot*Tstep;
    if abs(Vx)<vx_thres
        Vx=0;
    end
    if abs(Vy)<vy_thres
        Vy=0;
    end
    if abs(theta_dot)<theta_dot_thres
        theta_dot=0;
    end
    state_sequence(:,i)=[s;t;delta_theta;Vx;Vy;theta_dot;ax;ay];
end





end