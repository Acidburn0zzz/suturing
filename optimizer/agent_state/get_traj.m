function [ trajectory ] = get_traj( agent_state )
%Extracts the column vectory the corresponds to the robot trajectory.
    T = get_traj_length(agent_state);
    trajectory = agent_state(1:16*T);
end

