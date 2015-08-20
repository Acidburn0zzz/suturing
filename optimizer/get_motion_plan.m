function [ control_trajectory ] = get_motion_plan( env_state )
% Returns a list of 3d poses that specify the needle's trajectory.

% state initialization
agent_state = initialize_agent_state(env_state);

x0 = agent_state;
ineq_con = @(x)(0);
eq_con = @(x) equality_constraints(x, env_state);
f = @(x)(0);
% f = @(x)(cost_function(x, env_state));

control_trajectory = scp_lie_trajopt(f,x0,ineq_con, eq_con);
end