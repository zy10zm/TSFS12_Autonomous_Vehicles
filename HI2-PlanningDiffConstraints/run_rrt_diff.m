%% Run the RRT with a kinematic car model (two translational and one
%% orientational degree-of-freedom)

clear;
close all;
addpath Functions

%% Define world with obstacles

world = BoxWorld({[0, 10], [0, 10]});

%world.add_box(2, 2, 6, 6)
%world.add_box(1, 6, 4, 3)
%world.add_box(4, 1, 5, 4)

figure(10)
clf()
world.draw()
%axis([world.xmin, world.xmax, world.ymin, world.ymax])
axis([-2, 12, -2, 12])
xlabel('x')
ylabel('y');

start = [0; 0; 0]; % Start state (x,y,th)
goal = [9; 0; 0]; % Goal state (x,y,th)

% Define the possible control inputs
u_c = linspace(-pi/8, pi/8, 41);

% Define parameters and data structures

opts.beta = 0.2; % Probability for selecting goal state as target state
opts.delta = 0.3; % Step size (in time)
opts.eps = 0.01; % Threshold for stopping the search (negative for full search)
opts.K = 5000;    % Maximum number of iterations

%% Solve problem

fprintf('Planning ...\n');
[goal_idx, nodes, parents, state_trajectories, T] = rrt_diff(start, goal, u_c, @sim_car, world, opts);
fprintf('Finished in %.2f sek\n', T);

%% YOUR CODE HERE

% hold on
% figure(10)
% plot(nodes(1,:),nodes(2,:),('k.'))
% hold off

% Backtracking and plotting 
plan_idx = [goal_idx];
        while plan_idx(1) ~= parents(1)    
            plan_idx = [parents(plan_idx(1)) plan_idx];
        end
index = nan;
plan=[];
for i=1:numel(plan_idx)-1
    for j = 1:length(state_trajectories)
        if state_trajectories{2,j} == plan_idx(i) && state_trajectories{3,j} == plan_idx(i+1)
            index = j;
            break
        end
    end
    if ~isnan(index)
        correct_traj = state_trajectories{1,index};
        plan = [plan correct_traj];
    end
    index = nan;
end


% PLot av path
figure(10)
hold on
plot(plan(1,1), plan(2,1),'g+','LineWidth',5)
plot(goal(1,1), goal(2,1), 'b+')
plot(plan(1,end), plan(2,end),'r+','LineWidth',5)
plot(plan(1,:), plan(2,:),'r','LineWidth',2)
title('RRT with differential constraints')
hold off

%% Plot av orientation
figure(11)
plot(plan(3,:))
title('Orientation of the vehicle')
ylabel('rad')
xlabel('Position no.')

figure(10)
hold on
for i = 1:length(state_trajectories)
    plot(state_trajectories{1,i}(1,:),state_trajectories{1,i}(2,:),'b')
end
