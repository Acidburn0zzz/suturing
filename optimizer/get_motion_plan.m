function [ control_trajectory ] = get_motion_plan( env_state, timesteps )
% Returns a list of 3d points that specify the needle's trajectory.

% Trajectory initializations
agent_state = zeros(timesteps,3); % state specified by x,y,z at each timesteps
for i = 1:timesteps
    agent_state(i,:) = (1-i/timesteps)*env_state.start_point+(i/timesteps)*env_state.end_point;
end
A = eye(timesteps*3);
b = 2*ones(timesteps*3,1);

% set solver options
options = optimoptions('fmincon');
options.MaxFunEvals = 6000;
options.Algorithm = 'sqp';
Aeq = [];
Beq = [];
lb = -1*ones(timesteps*3,1);
ub = 1*ones(timesteps*3,1);
nonlcon = @nonlinear_constraints;
control_trajectory = fmincon(@(x)cost_function(x,env_state), agent_state, A,b, Aeq, Beq,lb,ub, nonlcon, options);
end