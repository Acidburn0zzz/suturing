function [ constraints ] = inequality_constraints(agent_state, env_state)
%   Detailed explanation goes here
    constraints = [lower_bounds(agent_state); signed_distance_constraints(agent_state, env_state)];
end


function [constraints] = signed_distance_constraints(agent_state, env_state)
% Returns the negative signed distance between poses and the mesh
    trajectory = get_traj(agent_state);
    get_position = @(i)(pos(get_traj_element(trajectory, i))');
    positions = arrayfun(get_position, 1:get_traj_length(agent_state), 'UniformOutput', false)';
    positions = cell2mat(positions);
    constraints = -1*signed_distance(positions, env_state.mesh.vertices, env_state.mesh.faces, 'SignedDistanceType', 'winding_number');
end

function [constraints] = lower_bounds(agent_state)
% Sets lower bounds for state
    constraints = -1*get_nontraj_elems(agent_state);
end