function [ pose ] = get_traj_element( trajectory, i )
%Returns the ith element of the trajectory
   pose = reshape(trajectory(16*i-15:16*i),4,4);
end

