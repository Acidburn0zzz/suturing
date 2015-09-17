function [ interpolated_traj ] = interpolate_traj( traj, sample_rate )
    % construct traj as position and quaternion
    n = length(traj)/16;
    traj_quat = zeros(7, n);
    for i = 1:n
       pose = get_traj_element(traj, i);
       position = pos(pose);
       quaternion = qGetQ(rot(pose));
       pose_vector = [position; quaternion];
       traj_quat(:,i) = pose_vector;
    end
    
    
    % interpolate trajectory
    x = 1:n;
    Y = traj_quat;
    xx = 1:1/sample_rate:n;
    YY = spline(x,Y,xx);

    interpolated_traj = zeros(16*n,1);
    % convert back from pos+quat to homogenous matrices
    a = size(YY);
    for i = 1:a(2)
        pose = YY(:,i);
        position = pose(1:3);
        quaternion = pose(4:7);
        new_pose = eye(4);
        new_pose(1:3,1:3) = qGetR(quaternion/norm(quaternion));
        new_pose(1:3,4) = position;
        
        interpolated_traj(16*i-15:16*i) = new_pose(:);
    end
end

