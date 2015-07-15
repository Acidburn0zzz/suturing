function [cost] = cost_function(agent_state, env_state)
% Returns a cost associated with a given state
    cost = 1*start_goal_cost(agent_state, env_state) + 1*end_goal_cost(agent_state, env_state) + trajectory_length_cost(agent_state);
end


function [cost] = start_goal_cost(state, env_state)
% trajectory should start near start goal
    traj_start = state(1,:);
    cost = norm(traj_start-env_state.start_point, 2);
end


function [cost] = end_goal_cost(state, env_state)
% trajectory should end near end goal
    traj_end = state(end,:);
    cost = norm(traj_end-env_state.end_point, 2);
end


function [cost] = trajectory_length_cost(state)
% favors trajectories that minimize length by returning l2 norm squared
% between consecutive points in trajectory
    state1 = [0 0 0; state];
    state2 = [state; 0 0 0];
    differences = (state1-state2);
    differences = differences(2:end-1,:);
    cost = sum(sum(differences.^2));
end

