function [cost] = cost_function(agent_state, env_state)
% Returns a cost associated with a given state
    cost = 100*start_goal_cost(agent_state, env_state) + ...
            100*end_goal_cost(agent_state, env_state) + ...
            trajectory_length_cost(agent_state, env_state);
end


function [cost] = start_goal_cost(agent_state, env_state)
% trajectory should start near start goal
    traj_start = get_traj_element(get_traj(agent_state), 1);
    cost = norm(pos(traj_start)-pos(env_state.start_pose), 2);
end


function [cost] = end_goal_cost(agent_state, env_state)
% trajectory should end near end goal
    traj_end = get_traj_element(get_traj(agent_state), env_state.T);
    cost = norm(pos(traj_end)-pos(env_state.end_pose), 2);
end


function [cost] = trajectory_length_cost(agent_state, env_state)
% favors trajectories that minimize length by returning the length of the
% trajectory
    delta = agent_state(end);
    cost = delta*(env_state.T-1);
end

