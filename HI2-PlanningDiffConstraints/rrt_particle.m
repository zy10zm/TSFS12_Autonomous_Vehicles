%% RRT for a particle moving in a plane (2D world)

function [goal_idx, nodes, parents, Tplan] = rrt_particle(start, goal, world, opts)

    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % world - description of the map of the world
    %         using an object from the class BoxWorld
    % opts - structure with options for the RRT
    
    % Output arguments:
    % goal_idx - index of the node closest to the desired goal state
    % nodes - 2 x N matrix with each column representing a state j
    %         in the tree
    % parents - 1 x N vector with the node number for the parent of node j 
    %           at element j in the vector (node number counted as column
    %           in the matrix nodes)
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = SampleFree()
        if rand < opts.beta
            x = goal;
        else
            foundRandom = false;
            while ~foundRandom
                x = [rand*(world.xmax-world.xmin) + world.xmin;...
                    rand*(world.ymax-world.ymin) + world.ymin];
                if world.ObstacleFree(x)
                    foundRandom = true;
                end
            end
        end
    end

    % Find index of state nearest to x in the matrix nodes
    function idx = Nearest(x)
        [~, idx] = min(sum((nodes-x).^2, 1));
    end

    % Steer from x1 towards x2 with step size opts.delta
    % 
    % If the distance to x2 is less than opts.delta, return
    % state x2.
    function x_new = Steer(x1, x2)
        if norm(x2 - x1) < opts.delta
            x_new = x2;
        else
            step = opts.delta;
            x_new = x1 + step*(x2 - x1)/norm(x2 - x1);
        end
    end

    % Start time measurement and define variables for nodes and parents
    tic;
    nodes = [start];
    parents = [1]; % Initial state has no parent
  
    % YOUR CODE HERE
    for i = 1:opts.K
        qrand = SampleFree(); % Får ut state [x y]
        qnearest_idx = Nearest(qrand);
        qnearest = [nodes(1,qnearest_idx);nodes(2,qnearest_idx)];
        qnew = Steer(qnearest,qrand);
        path = [linspace(qnew(1,1),qnearest(1,1),6);linspace(qnew(2,1),qnearest(2,1),6)];
        if ObstacleFree(world, path) 
            nodes = [nodes,qnew];
            parents = [parents,qnearest_idx];
        end
        
        if norm(qnew-goal) < opts.eps
            break
        end
    end
    Tplan = toc;
    [~, goal_idx] = min(sum((nodes - goal).^2, 1));
    
end