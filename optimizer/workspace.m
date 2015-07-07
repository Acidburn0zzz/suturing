%% Testing fmincon to generate simple paths around a sphere
clc
T = 20; % number of timesteps in trajectory
env_state = struct;
env_state.start_point = [-1 -1 -1]; % needle start point
env_state.end_point = [1 1 1]; % needle end point

trajectory = get_motion_plan(env_state, T);

%% Display generated trajectory
clf
X = trajectory(:,1);
Y = trajectory(:,2);

Z = trajectory(:,3);
h = scatter3(X, Y, Z);
h.MarkerFaceColor = [0 0.5 1];
hold on
[x,y,z] = sphere(10);
r = 0.4;
surf(r*x,r*y,r*z);
axis([-1 1 -1 1 -1 1]);
axis('square');