function [ timesteps ] = get_traj_length( agent_state )
%Returns number of timesteps in trajectory
    timesteps = (length(agent_state)-1)/16;
end

