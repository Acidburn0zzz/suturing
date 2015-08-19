function [ control_trajectory ] = get_motion_plan( env_state, timesteps )
% Returns a list of 3d poses that specify the needle's trajectory.

% Trajectory initialization
start_quat = qGetQ(env_state.start_pose(1:3,1:3));
end_quat = qGetQ(env_state.end_pose(1:3,1:3));
start_pos = env_state.start_pose(1:3,4);
end_pos = env_state.end_pose(1:3,4);

trajectory = zeros(timesteps*4,4);

for i = 1:timesteps
    position = (1-i/timesteps)*start_pos + (i/timesteps)*end_pos;
    rot_matrix = qGetR((1-i/timesteps)*start_quat + (i/timesteps)*end_quat);
    pose = eye(4);
    pose(1:3,1:3) = qGetR(start_quat);
    pose(1:3,4) = position + normrnd(0, 0.005, 3, 1);
    trajectory(4*i-3:4*i, :) = pose;
end


% eq_con = @(x)(0);
ineq_con = @(x)(0);
eq_con = @(x) equality_constraints(x, env_state);
% ineq_con = @(x) inequality_constraints(x, env_state);

agent_state = struct;
agent_state.trajectory = trajectory;
agent_state.delta = 0.2;



x0 = agent_state;
% f = @(x)cost_function(x,env_state);
f = @(x)(0);
control_trajectory = scp_lie_trajopt(f,x0,ineq_con, eq_con);
end