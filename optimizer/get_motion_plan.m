function [ control_trajectory ] = get_motion_plan( env_state, mesh, timesteps )
% Returns a list of 3d points that specify the needle's trajectory.

% Trajectory initializations
agent_state = zeros(timesteps,3); % state specified by x,y,z at each timesteps
for i = 1:timesteps
    agent_state(i,:) = (1-i/timesteps)*env_state.start_point+(i/timesteps)*env_state.end_point + normrnd(0,0.001);
end
A = eye(timesteps*3);
b = 2*ones(timesteps*3,1);

% set solver options
options = optimoptions('fmincon');
options.MaxFunEvals = 30000;
options.ScaleProblem = 'obj-and-constr';
% options.FinDiffRelStep = 2;
options.TypicalX = agent_state;
% options.Algorithm = 'sqp';
% options.Display = 'notify';

% interior points options
% options.AlwaysHonorConstraints = 'none';
% options.InitBarrierParam = 10000;

Aeq = [];
Beq = [];
lb = -1*ones(timesteps*3,1);
ub = 1*ones(timesteps*3,1);
nonlcon = @(x)nonlinear_constraints(x,mesh); 
% nonlcon = @nonlinear_constraints;
x0 = agent_state;
objective = @(x)cost_function(x,env_state);
control_trajectory = fmincon(objective, x0, A,b, Aeq, Beq,lb,ub, nonlcon, options);
end