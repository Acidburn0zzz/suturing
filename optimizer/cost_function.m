function [cost] = cost_function(agent_state, env_state)
% Returns a cost associated with a given state
    cost = start_goal_cost(agent_state, env_state)+end_goal_cost(agent_state, env_state);
end


function [cost] = start_goal_cost(state, env_state)
% trajectory should start near start goal
    traj_start = state(1:4,:);
    cost = norm(pos(traj_start)-pos(env_state.start_pose), 2);
end


function [cost] = end_goal_cost(state, env_state)
% trajectory should end near end goal
    traj_end = state(end-3:end,:);
    cost = norm(pos(traj_end)-pos(env_state.end_pose), 2);
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

