function [ constraints ] = inequality_constraints(agent_state, env_state)
%   Detailed explanation goes here
    constraints = [lower_bounds(agent_state)];
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
    result = get_nontraj_elems(agent_state);
    delta = result(1);
    curvature = result(2);
    curve_deltas = result(3:end);
    traj = get_traj(agent_state);
    pose = get_traj_element(traj, 5);
    position = pos(pose);
    depth = position(3);
%     delta_bounds = [0.2; 0.1; 0.05; 0.025; 0.0125; 0.0125/2];
    delta_bounds = 0.0001;
%     disp(curvature)
    constraints = [delta; -1*curvature; depth+0.3; curve_deltas-delta_bounds; -1*delta_bounds-curve_deltas];
%     constraints = -1*get_nontraj_elems(agent_state); % constraint and delta needs to be positive
end


function [constraints] = curvature_constraints(agent_state, env_state)
% Returns a list of equality constraints that ensure that the trajectory
% follows constant curvature path through the tissue.
    trajectory = get_traj(agent_state);
    elems = get_nontraj_elems(agent_state);
    delta = elems(1);
    curvature = elems(2);
    delta_curves = elems(3:end);
    e = 0.0;
    
%     twist = [0; delta; 0; delta*curvature; 0; 0];
    func_handler = @(x)(twistcoords(twistlog(get_traj_element(trajectory, x+1)*inv(twistexp([0; delta; 0; delta*(curvature+delta_curves(x)); 0; 0])*get_traj_element(trajectory, x)))));
    
    constraints = cell2mat(arrayfun(func_handler, 1:env_state.T-1, 'UniformOutput', false));
%     constraints_upper = constraints - repmat([0;delta;0;delta*(curvature+e);0;0],1,env_state.T-1);
%     constraints_lower = 1*repmat([0;delta;0;delta*(curvature-e);0;0],1,env_state.T-1)-constraints;
    constraints = constraints(:);
%     constraints = constraints/norm(constraints);
end