%% RRT for a model with a differential constraint defined by the input sim

% Example usage:
%      [goal_idx, nodes, parents, state_trajectories, Tplan] = 
%            rrt_diff(start, goal, u_c, @sim_car, world, opts);

function [goal_idx, nodes, parents, state_trajectories, Tplan] = ...
          rrt_diff(start, goal, u_c, sim, world, opts)
      
    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % u_c - vector with possible control actions (steering angles)
    % sim - function reference to the simulation model of the car motion
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
    % state_trajectories - a struct with the trajectory segment for reaching
    %                node j at element j (node number counted as column
    %                in the matrix nodes)
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = SampleFree()
        if rand < opts.beta
            x = goal;
        else
            foundRandom = false;
            th = rand*2*pi - pi;
            while ~foundRandom
                p = [rand*(world.xmax - world.xmin) + world.xmin;...
                    rand*(world.ymax - world.ymin) + world.ymin];
                if world.ObstacleFree(p)
                    foundRandom = true;
                    x = [p; th];
                end
            end
        end
    end

    % Find index of state nearest to x in nodes
    function idx = Nearest(x)
        [~, idx] = min(DistanceFcn(nodes, x));
    end

    % Compute all possible paths for different steering control signals u_c
    % to move from x_nearest towards x_rand, without collision
    %
    % If no valid paths are found, the returned variables are empty
    function [valid_new_paths, dist_to_x_rand] = SteerCandidates(x_nearest, x_rand)
        valid_new_paths = {};
        dist_to_x_rand = [];
        
        for k=1:length(u_c)
            p = sim(x_nearest, u_c(k), opts.delta);
            if world.ObstacleFree(p)
                valid_new_paths{end+1} = p;
                dist_to_x_rand(end+1) = DistanceFcn(p(:, end), x_rand);
            end
        end
    end

    % Function for computing the distance between states x1 and x2, 
    % where x1 can be a matrix with several state vectors, treating  
    % all states equally
%     function dist = DistanceFcn(x1, x2)
%             dist = sqrt(sum((x1 - x2).^2, 1));
%     end

     function dist = DistanceFcn(x1, x2)
            ang_dist = abs(x2(3,:)-x1(3,:));
            eu_dist = sqrt(sum((x1(1:2,:) - x2(1:2,:)).^2, 1)); 
            dist = eu_dist + 1*ang_dist;
     end

%      function dist = DistanceFcn(x1, x2)
%             
%             dist = sqrt(sum((x1(1:2,:) - x2(1:2,:)).^2, 1)); 
%            
%      end
 
    % Start time measurement and define variables for nodes, parents, and 
    % associated trajectories
    tic;
    nodes = [start];
    parents = [1];
    state_trajectories = {}; % No trajectory segment needed to reach start state
   
    
    % YOUR CODE HERE
    for i = 1:opts.K
        qrand = SampleFree(); % Får ut state [x; y; th]
        qnearest_idx = Nearest(qrand); % Hitta nodindex närmast samplad nod
        qnearest = [nodes(1,qnearest_idx);nodes(2,qnearest_idx);nodes(3,qnearest_idx)]; % koordinater för ovan
        [valid_trajectories,dist_to_rand] = SteerCandidates(qnearest,qrand); % Hittar giltiga trajektorier och avstånd från slutpunkt till samplad nod
        [min_dist,dist_idx] = min(dist_to_rand);  % Sorterar ut trajektorian som leder till kortast avstånd
        %if ObstacleFree(world, path) 
         %   nodes = [nodes,qnew];
          %  parents = [parents,qnearest_idx,u_c];
        %end
        if ~isempty(valid_trajectories) % Om det finns giltiga trajektorier
            trajectory = valid_trajectories{dist_idx}; % Plocka ut trajektorian med kortast avstånd till samplad nod
            tmp = {trajectory;0;0};
            state_trajectories = [state_trajectories tmp]; % Uppdatera listan med trajektorier
            qnew = trajectory(:,end); % Koordinater+vinkel för nya noden=slutpos för trajektorian
            nodes = [nodes,qnew]; % Lägg till ny nod i listan
            parents = [parents,qnearest_idx]; % Lägg till nya nodens parent
            state_trajectories{2,end} = qnearest_idx;
            state_trajectories{3,end} = length(nodes);
        end
        if norm(qnew(1:2,1)-goal(1:2,1)) < opts.eps
            break
        end
        
%         if DistanceFcn(qnew,goal) < opts.eps
%             break
%         end
%          hold on 
%          figure(10)
%          plot(trajectory(1,:), trajectory(2,:),'b','LineWidth',2)
    end
    
    Tplan = toc;
    [~, goal_idx] = min(DistanceFcn(nodes,goal));
end
