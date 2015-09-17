function [ ] = draw_traj( trajectory, color, alpha )
% Plot a trajectory as a sequence of poses
    timesteps = length(trajectory)/16; % number of timesteps in trajectory
    hold on
    for i = 1:timesteps
        pose = get_traj_element(trajectory, i);
        z = pose(1:3,3);
        y = pose(1:3,2);
        pose(1:3,2) = z;
        pose(1:3,3) = -y;
        
       x = pose(1:3,1);
       x = x/norm(x);
       start_point = pos(pose);
       scale = 0.2;
       end_point = start_point+scale*x;
       mArrow3(start_point, end_point, 'color', color, 'facealpha', alpha);
       
%         draw_pose(pose, 0.2, color)
%         drawframe(pose, 0.6)
    end
    grid on
    axis('equal');
    hold off
end

