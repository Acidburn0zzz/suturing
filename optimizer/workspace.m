%% Load Mesh
clc
[vertices, faces] = readSTL('meshes/Lower.STL');
incision_mesh.vertices = vertices;
incision_mesh.faces = faces;

mesh_edges = edges(incision_mesh.faces);

rotation = [0 0 1; 1 0 0; 0 1 0];
for i = 1:length(incision_mesh.vertices)
   incision_mesh.vertices(i,:) = (rotation*incision_mesh.vertices(i,:)' + [-0.3;-0.4;-0.4] )';
end
incision_mesh.vertices = 4*incision_mesh.vertices;
incision_mesh.vertices(:,3) = 2*incision_mesh.vertices(:,3);  % scale the height only

% scale = 0.005;
% scale_y = 1.0;
% incision_mesh.vertices = scale*incision_mesh.vertices;
% incision_mesh.vertices(:,3) = scale_y*incision_mesh.vertices(:,3);

%% Display Tissue Scene
clf
[vertices, faces] = readSTL('meshes/Lower.STL');
tissue = struct;
tissue.Vertices = vertices;
tissue.Faces = faces;
tissue.FaceColor = [1 0.1 0];
tissue.FaceAlpha = 0.1;
tissue.EdgeAlpha = 0.1;
patch(tissue);



%% Generate trajectory
clc
run('global_variables')

env_state = struct;
env_state.start_pose = start_pose;
env_state.end_pose = end_pose;
env_state.mesh = incision_mesh;
env_state.T = T;

agent_state = get_motion_plan(env_state);

%% Display trajectory

clf
traj = get_traj(agent_state);
interpolated_traj = interpolate_traj(traj, 1);
color = [0.0 0.5 0.0];
draw_traj(interpolated_traj, color);


hold on

% traj2 = get_traj(agent_mod);
% interpolated_traj = interpolate_traj(traj2, 2);
% draw_traj(interpolated_traj);


% % plot the mesh
hold on 

% load tissue
[vertices, faces] = readSTL('meshes/Lower.STL');
rotation = [0 0 1; 1 0 0; 0 1 0];
for i = 1:length(vertices)
   vertices(i,:) = (rotation*vertices(i,:)' + [-0.3;-0.4;-0.403] )';
end
vertices = 4*vertices;
vertices(:,3) = 2*vertices(:,3);  % scale the height only
tissue = struct;
tissue.Vertices = vertices;
tissue.Faces = faces;
tissue.FaceColor = [247/255 214/255 200/255];
tissue.FaceAlpha = 0.1;
tissue.EdgeColor = [0 0 0];
tissue.EdgeAlpha = 0.1;
patch(tissue);



% load stay out zone
[vertices, faces] = readSTL('meshes/Trap.STL');
rotation = [0 0 1; 1 0 0; 0 1 0];
for i = 1:length(vertices)
   vertices(i,:) = (rotation*vertices(i,:)' + [-0.3; -0.1; -0.1])';
end
vertices = 4*vertices;
vertices(:,3) = 1.0*vertices(:,3);  % scale the height only
stayout = struct;
stayout.Vertices = vertices;
stayout.Faces = faces;
stayout.FaceColor = [0 0 1];
stayout.FaceAlpha = 0.2;
stayout.EdgeColor = [0 0.0 1];
stayout.EdgeAlpha = 0.0;

patch(stayout)



grid off
axis([-2 2 -2 2 -2 2]);

%% Save Trajectory
traj = get_traj(agent_state);
interpolated_traj = interpolate_traj(traj, 5);
n = length(interpolated_traj)/16;
stacked = zeros(4*n, 4);
for i = 1:n
    stacked(4*i-3:4*i, :) = get_traj_element(interpolated_traj, i);
end

csvwrite('trajectory_no_pose.csv', stacked)

%% 
clf
traj = get_traj(circular_state);
traj2 = get_traj(variable_curvature);
traj3 = get_traj(less_constrained);

get_path_length(traj)
hold on
% draw_pose(get_traj_element(traj,1), 0.5, [1,0,0]);
% draw_pose(get_traj_element(traj2,1), 0.5,  [1,0,0]);
% 
% draw_pose(get_traj_element(traj,4), 0.5, [1,0,0]);
% draw_pose(get_traj_element(traj2,4),0.5,  [1,0,0]);
% 
% draw_pose(get_traj_element(traj,7), 0.5, [1,0,0]);
% draw_pose(get_traj_element(traj2,7), 0.5, [1,0,0]);

interpolated_traj = interpolate_traj(traj, 2);
draw_traj(interpolated_traj, [1,0,0], 0.0);
draw_traj(interpolated_traj, [0.8,0,0], 0.4);
interpolated_traj = interpolate_traj(traj2, 2);
draw_traj(interpolated_traj, [0,0,1], 1.0);
% draw_path(interpolated_traj);
interpolated_traj = interpolate_traj(traj3, 2);
draw_traj(interpolated_traj, [0,0.8, 0], 0.4);
% draw_path(interpolated_traj);



% draw_traj(traj2, [0,1,0])

% color = [0.0 0.5 0.0];
% draw_traj(interpolated_traj, color);
% 
% 
% hold on
% 
% traj = get_traj(circular_state);
% interpolated_traj = interpolate_traj(traj, 1);
% color = [0.0 0.5 0.0];
% draw_traj(interpolated_traj, color);


% % plot the mesh
hold on 

% load tissue
[vertices, faces] = readSTL('meshes/Lower.STL');
rotation = [0 0 1; 1 0 0; 0 1 0];
for i = 1:length(vertices)
   vertices(i,:) = (rotation*vertices(i,:)' + [-0.3;-0.4;-0.403] )';
end
vertices = 4*vertices;
vertices(:,3) = 2*vertices(:,3);  % scale the height only
tissue = struct;
tissue.Vertices = vertices;
tissue.Faces = faces;
tissue.FaceColor = [247/255/2 214/255/2 200/255/2];
tissue.FaceAlpha = 0.5;
tissue.EdgeColor = [0 0 0];
tissue.EdgeAlpha = 0.0;
patch(tissue);



% load stay out zone
[vertices, faces] = readSTL('meshes/Trap.STL');
rotation = [0 0 1; 1 0 0; 0 1 0];
for i = 1:length(vertices)
   vertices(i,:) = (rotation*vertices(i,:)' + [-0.3; -0.1; -0.1])';
end
vertices = 4*vertices;
vertices(:,3) = 1.0*vertices(:,3);  % scale the height only
stayout = struct;
stayout.Vertices = vertices;
stayout.Faces = faces;
stayout.FaceColor = [0 0 1];
stayout.FaceAlpha = 0.2;
stayout.EdgeColor = [0 0.0 0.0];
stayout.EdgeAlpha = 0.0;

patch(stayout)

grid off
axis([-2 2 -2 2 -2 2]);


