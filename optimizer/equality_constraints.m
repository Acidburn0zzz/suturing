function [ constraints ] = equality_constraints( agent_state, env_state )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    constraints = curvature_constraints(agent_state);

end

function [constraints] = curvature_constraints( agent_state )
% Returns a list of equality constraints that ensure that the trajectory
% follows constant curvature path through the tissue.
    delta = -0.25;
    curvature = 1;
%     twist = [delta; 0; 0; 0; delta*curvature; 0];
    twist = [0; delta; 0; delta*curvature; 0; 0];
    func_handler = @(x)(twistcoords(twistlog(agent_state(4*x+1:4*x+4,:)*inv(twistexp(twist)*agent_state(4*x-3:4*x,:)))));
%     func_handler = @(x)(agent_state(4*x+1:4*x+4,4)-agent_state(4*x-3:4*x,4)-[0.1; 0; 0; 0]);
    constraints = cell2mat(arrayfun(func_handler, 1:(size(agent_state)/4 -1), 'UniformOutput', false));
    constraints = 10*constraints(:);
end