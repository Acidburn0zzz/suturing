function [ elements ] = get_nontraj_elems( agent_state )
% Returns non trajectory elements
    T = get_traj_length(agent_state);
    elements = agent_state(16*T+1:end);
end

