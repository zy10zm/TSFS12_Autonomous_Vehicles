clear
close all
addpath Functions
% If you are in the student labs, run the line below to get access to
% CasADi
addpath("casadi-windows-matlabR2016a-v3.5.5")  

%%
filename = 'mprims.mat';
if exist(filename, 'file')
    mp = MotionPrimitives(filename);
else
    
    % Define the initial states and desired goal states for the motion
    % primitives
     theta_init = [0 pi/4 pi/2 3*pi/4 pi -3*pi/4 -pi/2 -pi/4];
     x_vec = [3 2 3 3 3 1 3 3 3 2 3];
     y_vec = [2 2 2 1 1 0 -1 -1 -2 -2 -2];
     th_vec = [0 pi/4 pi/2 0 pi/4 0 -pi/4 0 -pi/2 -pi/4 0];
     lattice = [x_vec;y_vec; th_vec];


    % Vehicle parameters and constraints
    L = 0.6138;        % Wheel base (m)
    v = 1;         % Constant velocity (m/s)
    u_max = pi/4;   % Maximum steering angle (rad)

    % Construct a MotionPrimitives object and generate the 
    % motion primitives using the constructed lattice and 
    % specification of the motion primitives
    mp = MotionPrimitives();
    mp.generate_primitives(theta_init, lattice, L, v, u_max);
    % Save the motion primitives to avoid unnecessary re-computation
    mp.save(filename);
end

%% Plot the computed primitives

figure(10)
clf()
mp.plot();
grid on;
xlabel('x');
ylabel('y');
axis('square');
box off
title('Motion Primitives');

%% Create world with obstacles using the BoxWorld class

xx = -2:1:12;
yy = -2:1:12;
th = [0 pi/4 pi/2 3*pi/4 pi -3*pi/4 -pi/2 -pi/4];
lattice = {xx, yy, th};

world = BoxWorld(lattice);

world.add_box(2, 2, 6, 6)
world.add_box(1, 6, 4, 3)
world.add_box(4, 1, 5, 4)

figure(10)
clf()
world.draw()
axis([world.xmin, world.xmax, world.ymin, world.ymax])
xlabel('x');
ylabel('y');

%%
start = [0; 0; 0]; % Initial state
goal = [10; 10; pi/2]; % Final state
%start = [10; 0; 0]; % Initial state
%goal = [0; 5; -pi/2]; % Final state


n = world.num_nodes();
eps = 1e-5;

% Define the initial and goal state for the graph search by finding the
% node number (column number in world.st_sp) in the world state space
mission.start.id = find(all(abs(world.st_sp - start) < eps, 1));
mission.goal.id = find(all(abs(world.st_sp - goal) < eps, 1));

% Define the function providing possible new states starting from x
f = @(x) next_state(x, world, mp);

% Define heuristics for cost-to-go
%cost_to_go = @(x, xg) norm(world.st_sp(1:2, x) - world.st_sp(1:2, xg));
c=10;
cost_to_go = @(x, xg) norm(world.st_sp(1:2, x) - world.st_sp(1:2, xg)) + c*abs(world.st_sp(3, x)-world.st_sp(3, xg));
% Solve problem using graph-search strategies from Hand-in Exercise 1

fprintf('Planning ...');
% Start planning
plan = {};
plan{end+1} = BreadthFirst(n, mission, f, [], 2);
plan{end+1} = DepthFirst(n, mission, f, [], 2);
plan{end+1} = Dijkstra(n, mission, f, [], 2);
plan{end+1} = AStar(n, mission, f, cost_to_go, 2);
plan{end+1} = BestFirst(n, mission, f, cost_to_go, 2);
fprintf('Done!\n');

opt_length = plan{3}.length; % Dijkstra is optimal

%% YOUR CODE HERE
hold on
path={};
for i=1:5    
    path{i} = control_to_path(mp, start, plan{1,i}.control); %Creates the path
    p(i) = plot(path{i}(1,:),path{i}(2,:),'color',rand(1,3),'linewidth',2) %Plots all the paths for the different alghoritms
    plot(world.st_sp(1,plan{1,i}.plan),world.st_sp(2,plan{1,i}.plan),'k*') %Plots the vistited nodes
    
    % Define necessary variables
    plan_length(i) = plan{i}.length;
    num_of_nodes(i) = plan{i}.num_visited_nodes;
    calc_time(i) = plan{i}.time;
end
legend([p(1),p(2),p(3),p(4),p(5)],'BreadthFirst','DepthFirst','Dijkstra','Astar','BestFirst')

%Plot plan lengths vs calculation time
figure(11)
plot(plan_length(1),calc_time(1),'k*',plan_length(2),calc_time(2),'r*',plan_length(3),calc_time(3),'b*',plan_length(4),calc_time(4),'m*',plan_length(5),calc_time(5),'g*');
xlabel('Plan length')
ylabel('Calculation time')
legend('BreadthFirst','DepthFirst','Dijkstra','Astar','BestFirst')

%Plot number of visited nodes vs calculation time
figure(12)
plot(num_of_nodes(1),calc_time(1),'k*',num_of_nodes(2),calc_time(2),'r*',num_of_nodes(3),calc_time(3),'b*',num_of_nodes(4),calc_time(4),'m*',num_of_nodes(5),calc_time(5),'g*');
xlabel('Number of nodes')
ylabel('Calculation time')
legend('BreadthFirst','DepthFirst','Dijkstra','Astar','BestFirst')