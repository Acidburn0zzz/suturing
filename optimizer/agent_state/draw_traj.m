function [ ] = draw_traj( trajectory )
% Plot a trajectory as a sequence of poses
    timesteps = length(trajectory)/16; % number of timesteps in trajectory
    hold on
    for i = 1:timesteps
        drawframe(get_traj_element(trajectory, i), 0.1)
    end
    grid on
    axis('equal');
    hold off

end

