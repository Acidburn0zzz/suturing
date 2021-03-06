function [ constraints ] = equality_constraints( agent_state, env_state )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    constraints = curvature_constraints(agent_state, env_state);
%     constraints = [];
end

% function [constraints] = curvature_constraints(agent_state, env_state)
% % Returns a list of equality constraints that ensure that the trajectory
% % follows constant curvature path through the tissue.
%     trajectory = get_traj(agent_state);
%     elems = get_nontraj_elems(agent_state);
%     delta = elems(1);
%     curvature = elems(2);
%     
%     twist = [0; delta; 0; delta*curvature; 0; 0];
%     func_handler = @(x)(norm(twistcoords(twistlog(get_traj_element(trajectory, x+1)*inv(twistexp(twist)*get_traj_element(trajectory, x))))- [0;0.05;0;0.05;0;0],2));
%     constraints = cell2mat(arrayfun(func_handler, 1:env_state.T-1, 'UniformOutput', false));
%     constraints = constraints(:);
% end


function [constraints] = curvature_constraints(agent_state, env_state)
% Returns a list of equality constraints that ensure that the trajectory
% follows constant curvature path through the tissue.
    trajectory = get_traj(agent_state);
    elems = get_nontraj_elems(agent_state);
    delta = elems(1);
    curvature = elems(2);
    delta_curves = elems(3:end);
    
%     twist = [0; delta; 0; delta*curvature; 0; 0];
    func_handler = @(x)(twistcoords(twistlog(get_traj_element(trajectory, x+1)/(twistexp([0; delta; 0; delta*(curvature+delta_curves(x)); 0; 0])*get_traj_element(trajectory, x)))));
    
    constraints = cell2mat(arrayfun(func_handler, 1:env_state.T-1, 'UniformOutput', false));
%     constraints_upper = constraints - repmat([0;delta;0;delta*(curvature+e);0;0],1,env_state.T-1);
%     constraints_lower = 1*repmat([0;delta;0;delta*(curvature-e);0;0],1,env_state.T-1)-constraints;
    constraints = constraints(:);
%     constraints = constraints/norm(constraints);
end