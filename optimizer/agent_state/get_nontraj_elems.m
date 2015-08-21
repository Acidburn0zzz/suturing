function [ elements ] = get_nontraj_elems( agent_state )
% Returns non trajectory elements
    elements = agent_state(end-1:end);
end

