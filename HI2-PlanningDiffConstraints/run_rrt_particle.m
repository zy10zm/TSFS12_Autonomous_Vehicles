%% Run the RRT for a particle moving in a plane (2D world)

clear;
close all;
addpath Functions

%% Define world with obstacles

world = BoxWorld({[0, 10], [0, 10]});
world.add_box(2, 2, 6, 6)
world.add_box(1, 6, 4, 3)
world.add_box(4, 1, 5, 4)


figure(10)
clf()
world.draw()
axis([world.xmin, world.xmax, world.ymin, world.ymax])
xlabel('x');
ylabel('y');

start = [1; 1];     % Start state
goal = [9; 9];      % Goal state

% Define parameters and data structures

opts.beta = 0.01; % Probability for selecting goal state as target state 
                  % in the sample
opts.delta = 0.1; % Step size
opts.eps = -3; % Threshold for stopping the search (negative for full search)
opts.K = 10000;    % Maximum number of iterations, if eps < 0

%% Solve problem

fprintf('Planning ...\n');
[goal_idx, nodes, parents, T] = rrt_particle(start, goal, world, opts);
fprintf('Finished in %.2f sek\n', T);

%% YOUR CODE HERE

tree = eye(numel(parents));

for i = 1:numel(parents)
    tree(i,parents(i)) = 1;
end
tree = tree' + tree;       
tree(1:1+size(tree,1):end) = 1;    

% Plotting the tree

nodes = nodes';

hold on
figure(10)
gplot(tree,nodes)
hold off

nodes = nodes';


% Backtracking and plotting
plan_idx = [goal_idx];
        while plan_idx(1) ~= parents(1)    
            plan_idx = [parents(plan_idx(1)) plan_idx];
        end

plan=[];
for i = 1:numel(plan_idx)
    plan = [plan [nodes(1,plan_idx(i)); nodes(2,plan_idx(i))]];
end

[~, num_of_nodes_tree] = size(nodes);
[~, num_of_nodes_plan] = size(plan);

plan_length=0;
for i = 1:length(plan)-1
    plan_length = plan_length + sqrt(sum((plan(:,i) - plan(:,i+1)).^2, 1));
end
    
    
figure(10)
hold on
plot(plan(1,1), plan(2,1),'g+','LineWidth',5)
plot(goal(1,1), goal(2,1), 'b+')
plot(plan(1,end), plan(2,end),'r+','LineWidth',5)
plot(plan(1,:), plan(2,:),'b','LineWidth',2)
hold off
title('RRT for particle model with step size 0.01')

