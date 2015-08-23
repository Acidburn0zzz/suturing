start_pose = eye(4);
start_pose(1:3,1:3) = [0 1 0; 0 0 -1; -1 0 0]; % set the orientation
start_pose(1:3,4) = [0 1 0]; % set the position

end_pose = eye(4);
end_pose(1:3,1:3) = [0 1 0; 0 0 1; 1 0 0]; % set the orientation
end_pose(1:3,4) = [0 -1 0]; % set the position

T = 7; % number of timesteps in trajectory