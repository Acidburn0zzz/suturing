function [  ] = draw_path( trajectory )
% Draws the suturing trajectory as a path
%   Detailed explanation goes here
    T = length(trajectory)/16;
    X = zeros(T,1);
    Y = zeros(T,1);
    Z = zeros(T,1);
    
    for i = 1:T
        pose = get_traj_element(trajectory, i);
        position = pos(pose);
        X(i) = position(1);
        Y(i) = position(2);
        Z(i) = position(3);
    end
    
    scatter3(X,Y,Z, 'LineWidth', 3);

end

