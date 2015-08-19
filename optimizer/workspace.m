%% Load Mesh
clc
[vertices, faces] = readSTL('meshes/test.stl');
incision_mesh.vertices = vertices;
incision_mesh.faces = faces;

mesh_edges = edges(incision_mesh.faces);
scale = 0.0025;
scale_y = 1;
incision_mesh.vertices = scale*incision_mesh.vertices;
incision_mesh.vertices(:,3) = scale_y*incision_mesh.vertices(:,3);

%% Generate trajectory
clc
run('global_variables')

env_state = struct;
env_state.start_pose = start_pose;
env_state.end_pose = end_pose;
env_state.mesh = incision_mesh;
env_state.T = T;

% trajectory = get_motion_plan(env_state, T);

%% Display trajectory

clf
hold on
drawframe(start_pose, 0.3);
drawframe(end_pose, 0.5);
% plot the trajectory
for i = 1:T
   drawframe(trajectory(4*i-3:4*i,:), 0.1)
end
% % plot the mesh
p = plot_edges(incision_mesh.vertices, mesh_edges);
for i = 1:numel(p)
    p(i).Color = [1 0 0];
    p(i).LineWidth = 1;
end
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
% axis('square');


%% Visualize Signed Distance field
n = 30;
step = 2/n;
points = zeros(n^3,3);
distances = zeros(n^3,1);
counter = 0;
for x = -1:step:1
    for y = -1:step:1
        for z = -1:step:1
            counter = counter + 1;            
            points(counter,:) = [x y z];
            distance = signed_distance([x y z], incision_mesh.vertices, incision_mesh.faces, 'SignedDistanceType', 'winding_number');
            distances(counter,:) = distance;
        end
    end
end
clf
plot = scatter3(points(:,1),points(:,2),points(:,3), [], distances);
plot.Marker = '.';
axis([-1 1 -1 1 -1 1]);
axis('square');

%% Test scp function
clc
f = @(x)sum(1.5.^(1:length(x))*sin(x));
eq_con = @(x)(5*pi-x(1));
ineq_con = @(x)(3*pi-x);
x0 = zeros(15,1);
solution = scp_solver(f,x0,ineq_con, eq_con);
disp(solution/pi)


%% Testing Agent State
agent_state = initialize_agent_state(env_state);
traj = get_traj(agent_state);
