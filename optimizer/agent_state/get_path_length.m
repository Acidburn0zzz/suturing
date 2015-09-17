function [ total  ] = get_path_length( traj )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    T = length(traj)/16;
    total = 0;
    for i = 1:T-1
        a = pos(get_traj_element(traj, i));
        b = pos(get_traj_element(traj, i+1));
        total = total + norm(b-a);
    end

end

