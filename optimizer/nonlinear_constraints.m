function [ceq] = nonlinear_constraints( agent_state, mesh )
%Returns true of false for constraints
% distance_constraints = signed_distance(agent_state, mesh.vertices, mesh.faces, 'SignedDistanceType', 'winding_number');
% ceq = curvature_constraints(agent_state);
end



function [constraints] = curvature_constraints( agent_state )
% Returns a list of equality constraints that ensure that the trajectory
% follows constant curvature path through the tissue.
    delta = 0.01;
    curvature = 0.1;
    twist = [delta; 0; 0; 0; delta*curvature; 0];
    func_handler = @(x)(twistcoords(twistlog(agent_state(4*x+1:4*x+4,:)*inv(twistexp(twist)*agent_state(4*x-3:4*x,:)))));
    constraints = cell2mat(arrayfun(func_handler, 1:(size(agent_state)/4 -1), 'UniformOutput', false));
    constraints = constraints(:);
end

% function [constraints] = separation_constraint(state)
% % constraints the maximum distance between points
%     max_distance = 0.1;
%     state1 = [0 0 0; state];
%     state2 = [state; 0 0 0];
%     differences = (state1-state2);
%     differences = abs(differences(2:end-1,:));
%     constraints = sum(sum(differences.^2)) - max_distance;
% end
% 
% function [constraints] = continuous_collision_constraints(state, mesh)
% % Uses ray mesh intersection between points to check for continuous
% % collisions    
%     state1 = [0 0 0; state];
%     state2 = [state; 0  0 0];
%     directions = state2-state1;
%     
%     directions = directions(2:end-1,:);
%     origins = state(1:end-1,:);
%     
%     origins2 = state(2:end,:);
%     directions2 = -directions;
%     
%     intersect_func =  @(x)sum(ray_mesh_intersect(origins(x,:),directions(x,:),mesh.vertices, mesh.faces));
%     intersect_func2 =  @(x)sum(ray_mesh_intersect(origins2(x,:),directions2(x,:),mesh.vertices, mesh.faces));
%     
%     n = size(origins);
%     n = n(1);
%     intersections = arrayfun(intersect_func, 1:n);
%     intersections2 = arrayfun(intersect_func2,1:n);
%     constraints = ~(intersections & intersections2) - 1.0;
% end
% 
% 
% function [constraints] = curvature_constraints(state)
% % contrains the curvature at every point in the trajectory
%     % curvature should be greate than min curvature at every point
%     min_curvature = 0.5;
%     max_curvature = 0.6;
%     func_handler = @(x)(local_curvature(state(x,:),state(x+1,:),state(x+2,:)));
%     func_handler2 = @(x)(local_curvature(state(x,:),state(x+2,:),state(x+4,:)));
%     curvatures = arrayfun(func_handler, 1:length(state)-3)';
%     curvatures2 = arrayfun(func_handler2, 1:length(state)-5)';
%     mid = floor((1+length(state))/2);
%     global_curvatures = local_curvature(state(1,:), state(mid,:), state(end,:));
%     curvatures = [curvatures; curvatures2; global_curvatures];
%     min_constraints = min_curvature - curvatures;
%     max_constraints = curvatures - max_curvature;
%     constraints = [min_constraints; max_constraints];
% end
% 
% function [curvature] = local_curvature(a,b,c)
%     A = triangleArea3d(a,b,c);
%     curvature = 4*A/(norm(a-b)*norm(b-c)*norm(c-a));
% end