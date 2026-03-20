function inst = allocate(inst)
% 为未分配的量测点分配新目标

age_max_idx = [];
age_max = 0;
for it = 1:inst.target_num_all
    if(inst.target(it).age > age_max)
        age_max = inst.target(it).age;
        age_max_idx = it;
    end
end
um = [];
if(~isempty(age_max_idx))
    if(inst.target(age_max_idx).age > 40)
        um = inst.target(age_max_idx).Hs;
    end
end

un = zeros(3,1);
uk = zeros(3,1);

for it1 = 1:inst.num_points
    if(inst.point_cloud(it1).best_ind > 0)
        continue;
    end
    new_target_ind = inst.target_num_all + 1;
    un(1) = inst.point_cloud(it1).range;
    un(2) = inst.point_cloud(it1).angle;
    un(3) = inst.point_cloud(it1).vel;
    alloc_num = 1;
    alloc_index = it1;
    un_sum = un;
    power_sum = inst.point_cloud(it1).power;
    for it2 = it1+1:inst.num_points
        if(inst.point_cloud(it2).best_ind > 0)
            continue;
        end
        uk(1) = inst.point_cloud(it2).range;
        uk(2) = inst.point_cloud(it2).angle;
        uk(3) = inst.point_cloud(it2).vel;
        %uk(3) = unroll_radvel(inst.velParams.maxradvel,un(3),inst.point_cloud(it2).vel);
        if(abs(uk(3)-un(3)) < inst.allocationParams.maxVelThre)
            dist = un(1)^2 + uk(1)^2 - 2*un(1)*uk(1)*cos(un(2)-uk(2));
            if(dist < inst.allocationParams.maxDistanceThre)
                alloc_index = [alloc_index it2];
                alloc_num = alloc_num + 1;
                un_sum(1) = un_sum(1) + uk(1);
                un_sum(2) = un_sum(2) + uk(2);
                un_sum(3) = un_sum(3) + uk(3);
                power_sum = power_sum + inst.point_cloud(it2).power;
                
                un(1) = un_sum(1)/alloc_num;
                un(2) = un_sum(2)/alloc_num;
                un(3) = un_sum(3)/alloc_num;
            end
        end
    end
    %新目标产生
    if(isempty(um))
        if(alloc_num > inst.allocationParams.pointsThre)
            for idx = alloc_index
                inst.point_cloud(idx).best_ind = new_target_ind;
            end
            power_mean = power_sum / alloc_num;
            inst.target_num_all = inst.target_num_all + 1;
            inst = target_start(inst,un,power_sum,power_mean);
        end
    else
        dist = un(1)^2 + um(1)^2 - 2*un(1)*um(1)*cos(un(2)-um(2));
        if(alloc_num > inst.allocationParams.pointsThre & ...
                dist > 0.5)
            for idx = alloc_index
                inst.point_cloud(idx).best_ind = new_target_ind;
            end
            power_mean = power_sum / alloc_num;
            inst.target_num_all = inst.target_num_all + 1;
            inst = target_start(inst,un,power_sum,power_mean);
        end
    end
end