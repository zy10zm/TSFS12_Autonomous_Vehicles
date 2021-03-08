function [xi, u, d] = next_state(x, world, mp, tol)

    % Input arguments:
    % x - current state
    % world - description of the map of the world
    %         using the class BoxWorld
    % mp - object with motion primitives of the class MotionPrimitives
    % tol - tolerance for comparison of closeness of states
    
    % Output arguments:
    % xi - 3 x N matrix containing the possible next states from current 
    %       state x, considering the obstacles and size of the world model
    % u - N x 2 matrix with the indices of the motion primitives used for
    %     reaching each state (row in u corresponds to column in xi)
    % d - 1 x N vector with the cost associated with each possible 
    %     transition in xi 
    
    if nargin < 4
        tol = 1e-5;
    end
    xi = [];
    u = [];
    d = [];

    x_state = world.st_sp(:, x);

    % Extract the set of primitives corresponding to the current angle state
    k = find(abs(mp.theta_init - x_state(3)) < tol);
    mprims_x = mp.mprims{k};
    
    % Iterate through all available primitives
    for j = 1:length(mprims_x)
        mpj = mprims_x{j};

        % Create path to next state
        p = x_state(1:2) + [mpj.x; mpj.y];
        state_next(1:2, 1) = p(1:2, end);
        state_next(3, 1) = mpj.th(end);
        
        % Check if the path to next state is in the allowed area
        if ~world.in_bound(state_next) || ~world.ObstacleFree(p)
            continue;
        else
            xi = [xi find(all(abs(world.st_sp - state_next) < tol, 1))];
            u = [u; [k j]];
            d = [d mprims_x{j}.ds];
        end
    end
end
