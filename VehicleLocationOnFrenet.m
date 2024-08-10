function [st_frenet,road_condition,closest_ind,pre_ind] = VehicleLocationOnFrenet(vehicle_state,path_road,Ld,ind_last)
N=length(path_road);
n=50;m=50;
distance=zeros(1,N);
begin_flag=1;
end_flag=N;
if nargin<4
    begin_flag=2;
    end_flag=1;
else
    if ind_last-n>0&&ind_last+m<N
        begin_flag=ind_last-n;
        end_flag=ind_last+m-1;
    elseif ind_last-n<1&&ind_last+m<N
        begin_flag=1;
        end_flag=ind_last+m-1;
    elseif ind_last-n>0&&ind_last+m>N
        begin_flag=ind_last-n;
        end_flag=N;
    end
end
if begin_flag>end_flag
    begin_flag=1;
    end_flag=N;
end
for i=begin_flag:end_flag
    distance(i)=norm(vehicle_state(1:2)-path_road(1:2,i));
%     distance(i)=norm(vehicle_state(1)-path_road(1,i));
end
[~,closest_ind]=min(distance(begin_flag:end_flag));
closest_ind=closest_ind+begin_flag-1;
pre_ind=closest_ind;
s_frenet=path_road(end,closest_ind);
delta_theta=vehicle_state(3)-path_road(3,closest_ind);
t_frenet=-(vehicle_state(1)-path_road(1,closest_ind))*sin(path_road(3,closest_ind))+(vehicle_state(2)-path_road(2,closest_ind))*cos(path_road(3,closest_ind));
st_frenet=[s_frenet;t_frenet;delta_theta];%s,t,delta_theta
road_condition=[path_road(6,closest_ind);path_road(4,closest_ind)];%phi,curvature

L=0;
while L<Ld&&pre_ind<N
    L=L+norm(path_road(1:2,pre_ind+1)-path_road(1:2,pre_ind));
    pre_ind=pre_ind+1;
end
pre_ind=min(pre_ind,N);




end