function [ constraints ] = inequality_constraints(agent_state, env_state)
%   Detailed explanation goes here
    constraints = max(signed_distance_constraints(agent_state, env_state));
end


function [constraints] = signed_distance_constraints(agent_state, env_state)
% Returns the negative signed distance between poses and the mesh
    trajectory = agent_state;
    get_position = @(i)(trajectory(4*i-3:4*i-1,4)');
    positions = arrayfun(get_position, 1:length(trajectory)/4, 'UniformOutput', false)';
    positions = cell2mat(positions);
    constraints = -1*signed_distance(positions, env_state.mesh.vertices, env_state.mesh.faces, 'SignedDistanceType', 'winding_number');
end