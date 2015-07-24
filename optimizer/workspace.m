%% Load Mesh
clc
[vertices, faces] = readSTL('~/Documents/research/suturing/code_base/meshes/test.stl');
incision_mesh.vertices = vertices;
incision_mesh.faces = faces;

mesh_edges = edges(incision_mesh.faces);
scale = 0.0025;
scale_y = 1;
incision_mesh.vertices = scale*incision_mesh.vertices;
incision_mesh.vertices(:,3) = scale_y*incision_mesh.vertices(:,3);

%% Generate trajectory
T = 20; % number of timesteps in trajectory
env_state = struct;

start_pose = eye(4);
start_pose(1:3,1:3) = [0 1 0; 0 0 -1; -1 0 0]; % set the orientation
start_pose(1:3,4) = [0 1 0]; % set the position

end_pose = eye(4);
end_pose(1:3,1:3) = [0 1 0; 0 0 1; 1 0 0]; % set the orientation
end_pose(1:3,4) = [0 -1 0]; % set the position

env_state.start_pose = start_pose;
env_state.end_pose = end_pose;

trajectory = get_motion_plan(env_state, incision_mesh, T);

%% Display trajectory

clf
hold on
for i = 1:T
   drawframe(trajectory(4*i-3:4*i,:), 0.25)
end

p = plot_edges(incision_mesh.vertices, mesh_edges);
for i = 1:numel(p)
    p(i).Color = [1 0 0];
    p(i).LineWidth = 1;
end

axis([-1 1 -1 1 -1 1]);
axis('square');


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
% f = @(x)((x(1))*(x(1))+x(2)*x(2));
f  = @(x)(x(1)*x(1) + x(2)*x(2) + x(3)*x(3) + x(4)*x(4) );
f2  = @(x)(x(1) + x(2) + x(3) + x(4) );
eq_con = @(x)(0);
ineq_con = @(x)(0);
ineq_con2 = @(x)([3-x(1)*10000; 9-x(2)*10; x-1; x-1; -x]);
ineq_con3 = @(x)([1000-x(1); 100-x(2); 10-x(3); 1-x(4)]);
x0 = [200; 200; 200; 200];
solution = scp_lie_trajopt(f,x0,ineq_con3, eq_con);
disp(solution)


