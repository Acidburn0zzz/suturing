function [ c, ceq ] = nonlinear_constraints( agent_state )
%Returns true of false for constraints
ceq = [];
c = (0.16-sum(agent_state.^2,2));
end

