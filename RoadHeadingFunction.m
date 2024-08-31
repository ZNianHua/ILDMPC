function [heading_sequence] = RoadHeadingFunction(R_list,end_list,station_sequence)
num_road=length(R_list);
heading_sequence=0.*station_sequence;
heading_last=0;
index_last=0;
for i=1:num_road
    end_point=end_list(i);
    [~,min_ind1]=min(abs(station_sequence(index_last+1:end)-end_point));
    min_index=min_ind1+index_last;
    for j=min_index-10:min_index+10
        if station_sequence(j)-end_point<0 && station_sequence(j+1)-end_point>=0
            selected_index=j;
            break
        end
    end
    if i==1
        original_station=0;
    else
        original_station=station_sequence(index_last);
    end
    if R_list(i)==0
        curvature=0;
    else
        curvature=1/R_list(i);
    end
    heading_sequence(index_last+1:selected_index)=heading_last+curvature.*(station_sequence(index_last+1:selected_index)-original_station);
    index_last=selected_index;
    heading_last=heading_sequence(selected_index);
end
end