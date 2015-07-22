function [ trajectory ] = sqp_lie_trajopt(f, x0, ineq_con, eq_con)
%Returns a locally optimal trajectory
%   This method uses sequential convex programming and optimize on the
%   manifold using the lie algebra.
    
    %===========
    % OPTIONS  
    %===========??
    params = struct;
    params.mu = 0.01; % initial penalty coefficient
    params.k = 2; % penalty scaling factor
    params.s = 0.1; % intial trust region size
    params.trust_region_scale = 2; % step size to scale the trust region
    params.min_approx_quality = 0.1; % minimum approximation quality for the trust region
    params.max_penalty_iterations = 100; % number of iterations penalty loop
    params.max_convex_iterations = 1000;
    params.max_trust_region_iterations = 1000;
    params.max_ctol = 0.001; % max_deviation for constraints
    params.max_xtol = 0.001;
    params.max_ftol = 0.001;
    params.h = 0.00001; % difference used to compute numerical gradients

    % penalty iteration loop
    x = x0;
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
            params.mu = params.mu*params.k;
        end
    end
    if con_tol > params.max_ctol
        disp('Failed to satisfy constraints');
    end
    trajectory = x; 
    
end


function [trajectory, con_tol] = opt_unconstrained(f, x, ineq_con, eq_con, params)
    s = params.s; % get the initial trust region size
    for i = 1:params.max_convex_iterations
        % generate local convex approximation of the problem
        mu = params.mu;
        % construct the lagrangian
        lagrangian = @(x)(f(x) + mu*sum(max(0,ineq_con(x))) + mu*sum(abs(eq_con(x))));
        
        % compute the gradient numerically
        gradient = numerical_gradient(lagrangian, x, params.h);
        
        
        % solve local linear problem
        curr_obj = lagrangian(x); % current value of the objective function
        for j = 1:params.max_trust_region_iterations
           x_next = x - s*gradient;
           next_obj = lagrangian(x_next);
           quality = (curr_obj-next_obj)/(s*(gradient'*gradient));
           if quality > params.min_approx_quality % quality of the approximation is good
               x = x_next;
               s = params.trust_region_scale*s; % increase the size of the trust region
               break;
           else
               s = s/params.trust_region_scale; % decrease the size of the trust region
           end
           
           % check if the step size is smaller than the tolerance
           if s < params.max_xtol
               break;
           end
        end
        
        % check if objective or state is converging
        if s < params.max_xtol || curr_obj-next_obj < params.max_ftol
            break
        end
    end
    trajectory = x;
    con_tol = max(max(eq_con(x)), max(ineq_con(x))); % compute the largest constraint violation
end


function [gradient] = numerical_gradient(f, x, h)
    eval_points = repmat(x,1,length(x)) + h*eye(length(x));
    array_func_handler = @(x)(f(eval_points(:,x)));
    delta_f = arrayfun(array_func_handler, 1:length(x))' - f(x);
    gradient = delta_f/h;
end
