function [ trajectory ] = get_traj( agent_state )
%Extracts the column vectory the corresponds to the robot trajectory.
    trajectory = agent_state(1:end-2);
end

