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
env_state.start_point = [0 -1 0.7]; % needle start point
env_state.end_point = [0 1 0.7]; % needle end point

trajectory = get_motion_plan(env_state, incision_mesh, T);

%% Display generated trajectory
clf
X = trajectory(:,1);
Y = trajectory(:,2);
Z = trajectory(:,3);

h = scatter3(X, Y, Z);
h.MarkerFaceColor = [0 0.5 1];

hold on

% use this for a nicer mesh
% drawMesh(incision_mesh.vertices, incision_mesh.faces, [1 0 0])

p = plot_edges(incision_mesh.vertices, mesh_edges);
for i = 1:numel(p)
    p(i).Color = [1 0 0];
    p(i).LineWidth = 1;
end
plot3(X,Y,Z);

xlabel('x');
ylabel('y');
zlabel('z');
axis([-1 1 -1 1 -1 1]);
axis('square');

%% Visualize Signed Distance field
n = 40;
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