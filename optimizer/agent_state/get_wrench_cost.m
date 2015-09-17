function [ cost ] = get_wrench_cost( agent_state )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    T = get_traj_length(agent_state);
    traj = get_traj(agent_state);
    
    curve_deltas = agent_state(end-5: end);
    
    curr_traj_length = 0;
    wrench_cost = 0;
    for i = 1:T-1
        a = pos(get_traj_element(traj, i));
        b = pos(get_traj_element(traj, i+1));
        curr_traj_length = curr_traj_length + norm(b-a);
        wrench_cost = wrench_cost + curve_deltas(i)*curr_traj_length;
    end
    cost = wrench_cost;
end

