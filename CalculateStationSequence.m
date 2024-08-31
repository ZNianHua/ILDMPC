function [s_frenet] = CalculateStationSequence(path_xyz)
[n,m]=size(path_xyz);
if n<m
    path_use=path_xyz;
    s_frenet=zeros(1,m);
    for i=2:m
        s_frenet(i)=norm(path_use(:,i)-path_use(:,i-1))+s_frenet(i-1);
    end
else
    path_use=path_xyz';
    s_frenet=zeros(1,n);
    for i=2:m
        s_frenet(i)=norm(path_use(:,i)-path_use(:,i-1))+s_frenet(i-1);
    end
end

end