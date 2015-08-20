function [ agent_state ] = scp_lie_trajopt(f, agent_state, ineq_con, eq_con)
%Returns a locally optimal agent_state
%   This method uses sequential convex programming and optimize on the
%   manifold using the lie algebra.

    %.===========
    % OPTIONS
    %============
    params = struct;
    params.mu = 1; % initial penalty coefficient
    params.k = 2; % penalty scaling factor
    params.s = 0.5; % intial trust region size
    params.trust_region_scale = 1.02; % step size to scale the trust region
    params.trust_region_scale2 = 1.01;
    params.min_approx_quality = 0.5; % minimum approximation quality for the trust region
    params.max_penalty_iterations = 1; % number of iterations penalty loop
    params.max_convex_iterations = 500;
    params.max_trust_region_iterations = 2000;
    params.max_ctol = 0.0001; % max_deviation for constraints
    params.max_xtol = 0.000001;
    params.max_ftol = 0.000001;
    params.h = 0.00001; % difference used to compute numerical gradients

    % penalty iteration loop
    x = agent_state;
    for i = 1:params.max_penalty_iterations
        % solve the unconstrained problem
        [x, con_tol] = opt_unconstrained(f,x,ineq_con, eq_con, params);
        % check if the constraints are violated
        if con_tol <= params.max_ctol
            % the constraints are not violated, solution is found
            break
        else
            % constraints are violated so the penalty coefficent must
            % be increased
            disp('Constraint penalty too small. Re-solving with higher penalities');
            params.mu = params.mu*params.k;
        end
    end
    if con_tol > params.max_ctol
        disp('Failed to satisfy constraints');
    end
    agent_state = x;
end


function [agent_state, con_tol] = opt_unconstrained(f, x, ineq_con, eq_con, params)

    timesteps = get_traj_length(x);
    s = params.s; % get the initial trust region size
    is_increasing = false;
    for i = 1:params.max_convex_iterations

        % generate local convex approximation of the problem
        mu = params.mu;
        % construct the lagrangian
        lagrangian = @(x)(f(x) + mu*sum(max(0,ineq_con(x))) + mu*sum(abs(eq_con(x))));

        % initialize local parameterization to zero
        lie_x = [zeros(6*timesteps,1); get_nontraj_elems(x)];

        % lagrangian as a lie algebra
        lie_lagrangian = generate_lie_function(lagrangian, x);

        % compute the gradient numerically
        lie_gradient = numerical_gradient(lie_lagrangian, lie_x, params.h);

        % solve local linear problem
        curr_obj = lagrangian(x); % current value of the objective function
        disp(curr_obj)

        counter = 1;

        for j = 1:params.max_trust_region_iterations
           lie_x_next = lie_x-s*lie_gradient;
           next_obj = lie_lagrangian(lie_x_next);
           quality = (curr_obj-next_obj)/s;

           if quality > params.min_approx_quality % quality of the approximation is good

               x = apply_twists(x, lie_x_next); % update x to new value

               s = s*(params.trust_region_scale); % increase the size of the trust region
               if is_increasing == true
                   counter = counter + 1;
               else
                   counter = 1;
               end

               break;
           else
               s = s/(params.trust_region_scale2); % decrease the size of the trust region
               is_increasing = false;

               if is_increasing == false
                   counter = counter + 1;
               else
                   counter = 1;
               end

           end

           % check if the step size is smaller than the tolerance
           if s < params.max_xtol
               break;
           end
        end

        % check if objective or state is converging
        if s < params.max_xtol
            disp('Breaking out of SQP loop: X steps size too small')
            break
        end
        if curr_obj-next_obj < params.max_ftol
            disp('Breaking out of SQP loop: objective improvement too small')
            break
        end
    end
    agent_state = x;
    con_tol = max(max(abs(eq_con(x))), max(ineq_con(x))); % compute the largest constraint violation
end


function [gradient] = numerical_gradient(f, x, h)

    delta_f = zeros(length(x),1);
    for i = 1:length(x)
        point = x;
        point(i) = point(i) + h;
        delta_f(i) = f(point);
    end
    delta_f = delta_f - f(x);
    gradient = delta_f/norm(delta_f);

%     eval_points = repmat(x,1,length(x)) + h*eye(length(x));
%     array_func_handler = @(x)(f(eval_points(:,x)));

%     delta_f = arrayfun(array_func_handler, 1:length(x))'-f(x);
%     gradient = delta_f/norm(delta_f);
%     eval_points2 = repmat(x,1,length(x)) + h*eye(length(x));
%     eval_points1 = repmat(x,1,length(x)) - h*eye(length(x));
%     array_func_handler2 = @(x)(f(eval_points2(:,x)));
%     array_func_handler1 = @(x)(f(eval_points1(:,x)));
%     delta_f = arrayfun(array_func_handler2, 1:length(x))'-arrayfun(array_func_handler1, 1:length(x))';
%     gradient = delta_f/norm(delta_f);

end

function [lie_function] = generate_lie_function(f, agent_state)
% Given a scalar function f and a homogenous(4x4) coordinate x this method
% returns a scalar function that is a function of the local lie algebra.
% TODO: Write a better function description
    lie_function = @(lie_agent_state)(f(apply_twists(agent_state, lie_agent_state)));
end

function [output_agent_state] = apply_twists(input_agent_state, lie_agent_state)
% Apply lie_twists to agent state
    input_traj = get_traj(input_agent_state);
    timesteps = get_traj_length(input_agent_state);
    apply_twist = @(i)(reshape(twistexp(lie_agent_state(6*i-5:6*i))*get_traj_element(input_traj, i), [], 1)); 
    output_traj = arrayfun(apply_twist, 1:timesteps, 'UniformOutput', false);
    output_traj = cell2mat(output_traj');
    output_agent_state = [output_traj(:); lie_agent_state(6*timesteps+1:end)];
end
