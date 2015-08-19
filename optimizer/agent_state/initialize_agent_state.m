function [ agent_state ] = initialize_agent_state(start_pose, end_pose, timesteps)
% Returns an initial agent/robot state.
%   The method returns a column vector containing the robot's trajectory
%   and other parameters. The method uses linear interpolation to
%   initialize a trajectory from the start pose to the end pose.
    
    trajectory = zeros(16*timesteps,1);
    
    trajectory(1:16) = start_pose(:); % first point is just the start pose
    
    
    start_quaternion = qGetQ(rot(start_pose));
    end_quaternion = qGetQ(rot(end_pose));
    start_position = pos(start_pose);
    end_position = pos(end_pose);
    
    step = 1/(timesteps-1);
    for i = 1:timesteps-1
        % interpolate position and rotation
        position = (1-step*i)*start_position + (step*i)*end_position;
        quaternion = (1-step*i)*start_quaternion + (step*i)*end_quaternion;
        
        % construct pose
        pose = eye(4);
        pose(1:3,1:3) = qGetR(quaternion/norm(quaternion));
        pose(1:3,4) = position;
        
        % update trajectory with flattened pose
        trajectory(16*i+1:16*i+16) = pose(:);
    end
    
    % add parameter for spacing between trajectory points
    delta = norm(end_position-start_position)/(timesteps-1);
    
    % concatenate additional parameters to the end of the flattened
    % trajectory
    agent_state = [trajectory; delta];
end
