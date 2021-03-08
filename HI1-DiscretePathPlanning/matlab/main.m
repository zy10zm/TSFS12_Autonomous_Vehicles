%% TSFS12 Hand-in exercise 1: Discrete planning in structured road networks
clear
close all

addpath Functions

%% Read map information
mapdir = '../Maps/';
mapfile = strcat(mapdir, 'linkoping.osm'); 
figurefile = strcat(mapdir, 'linkoping.png');
map = loadOSMmap(mapfile, figurefile);

num_nodes = map.nodes.size(1); % Number of nodes in the map
%% Display basic information about the map
map.info()

% Plot the map
figure(10)
map.plotmap()
xlabel('Longitude (^o)')
ylabel('Latitude (^o)')
title('Linköping')

% Which nodes are neighbors to node number 111?
map.neighbors(111)

% What are the distances (in m) between nodes 111-3401 and 111-3402?
full(map.distancematrix(111, 3401))

% What is the position in latitude and longitude of node 3401
pos = map.nodeposition(3401);
fprintf('Longitude=%.2f, Latitude=%.2f\n', pos(1), pos(2));

% Plot the distance matrix and illustrate sparseness
figure(20)
spy(map.distancematrix>0)
density = full(sum(sum(map.distancematrix>0)))/(num_nodes^2);
title(sprintf('Density %.2f%%', density*100));
xlabel('Node index');
ylabel('Node index');

%% Some pre-defined missions to experiment with. 
% Example, the first mission can be used in a planner with
%   planner(num_nodes, pre_mission{1}, f_next, cost_to_go)

pre_mission = {...
    struct('start', struct('id', 10907), 'goal', struct('id', 1025)), ...
    struct('start', struct('id', 3988), 'goal', struct('id', 4725)), ...
    struct('start', struct('id', 424), 'goal', struct('id', 365))};


%% Define planning mission
figure(30)
map.plotmap()
title('Linköping - click in map to define mission')
hold on
mission.start = map.getmapposition();
plot(mission.start.pos(1), mission.start.pos(2), 'bd', 'MarkerSize', 10);

mission.goal = map.getmapposition();
plot(mission.goal.pos(1), mission.goal.pos(2), 'bd', 'MarkerSize', 10);
hold off

% Inspect the defined mission
mission.start
mission.goal


%% Define state update function. Here, the function computes neighbouring nodes
% and the corresponding distance
f_next = @(x) map_state_update(x, map.distancematrix);

%% Define heuristic for Astar and BestFirst
% Hint: Function LatLongDistance is useful
cost_to_go = @(x, xg) LatLongDistance(map.nodeposition(x),map.nodeposition(xg));

%% Running the planners
mission = pre_mission{1}; % Choose mission here, or create your own above.

df_plan = DepthFirst(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    df_plan.length, df_plan.num_visited_nodes, df_plan.time*1000);

bf_plan = BreadthFirst(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    bf_plan.length, bf_plan.num_visited_nodes, bf_plan.time*1000);

dijkstra_plan = Dijkstra(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    dijkstra_plan.length, dijkstra_plan.num_visited_nodes, dijkstra_plan.time*1000);

astar_plan = AStar(num_nodes, mission, f_next, cost_to_go);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    astar_plan.length, astar_plan.num_visited_nodes, astar_plan.time*1000);

bestfirst_plan = BestFirst(num_nodes, mission, f_next, cost_to_go);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
   bestfirst_plan.length, bestfirst_plan.num_visited_nodes, bestfirst_plan.time*1000);
%% Plotting the plans

% Plot the resulting plan of using DepthFirst
figure(1)
map.plotmap()
hold on
map.plotplan(df_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', df_plan.name, df_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Planned path")

% Plot visited nodes during search with DepthFirst
figure(2)
map.plotmap()
hold on
map.plotplan(df_plan.visited_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', df_plan.name, df_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Visited nodes")

% Plot the resulting plan of using BreadthFirst
figure(3)
map.plotmap()
hold on
map.plotplan(bf_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', bf_plan.name, bf_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Planned path")

% Plot visited nodes during search with BreadthFirst
figure(4)
map.plotmap()
hold on
map.plotplan(df_plan.visited_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', bf_plan.name, bf_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Visited nodes")

% Plot the resulting plan of using Dijkstra
figure(5)
map.plotmap()
hold on
map.plotplan(dijkstra_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', dijkstra_plan.name, dijkstra_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Planned path")

% Plot visited nodes during search with Dijsktra
figure(6)
map.plotmap()
hold on
map.plotplan(dijkstra_plan.visited_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', dijkstra_plan.name, dijkstra_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Visited nodes")

% Plot the resulting plan of using A*
figure(7)
map.plotmap()
hold on
map.plotplan(astar_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', astar_plan.name, astar_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Planned path")

% Plot visited nodes during search with A*
figure(8)
map.plotmap()
hold on
map.plotplan(astar_plan.visited_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', astar_plan.name, astar_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Visited nodes")

% Plot the resulting plan of using BestFirst
figure(9)
map.plotmap()
hold on
map.plotplan(bestfirst_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', bestfirst_plan.name, bestfirst_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Planned path")

% Plot visited nodes during search with BestFirst
figure(10)
map.plotmap()
hold on
map.plotplan(bestfirst_plan.visited_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', bestfirst_plan.name, bestfirst_plan.length));
xlabel("Longitude" + "(^o)")
ylabel("Latitude" + "(^o)")
title("Visited nodes")

% Plotting all of the resulting plans
figure(40)
map.plotmap()
title('Planned road')
hold on
map.plotplan(df_plan.plan, 'b', 'linewidth', 2);
map.plotplan(bf_plan.plan, 'r', 'linewidth', 2);
map.plotplan(dijkstra_plan.plan, 'g', 'linewidth', 2);
map.plotplan(bestfirst_plan.plan, 'k', 'linewidth', 2);
map.plotplan(astar_plan.plan, 'c', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', df_plan.name, df_plan.length),sprintf('%s (%.1f m)',...
    bf_plan.name, bf_plan.length),sprintf('%s (%.1f m)',...
    dijkstra_plan.name, dijkstra_plan.length),sprintf('%s (%.1f m)',...
    bestfirst_plan.name, bestfirst_plan.length),sprintf('%s (%.1f m)',...
    astar_plan.name, astar_plan.length),'Location','SouthEast');
xlabel('Longitude' + "(^o)")
ylabel('Latitude' + "(^o)")
title("Planned paths")


%% Plotting reflecting figures

figure(11)
plot(bf_plan.time.*1000,bf_plan.num_visited_nodes,'*', dijkstra_plan.time*1000,dijkstra_plan.num_visited_nodes,'*', ...
    astar_plan.time.*1000,astar_plan.num_visited_nodes,'*',bestfirst_plan.time.*1000,bestfirst_plan.num_visited_nodes,'*')
xlabel('Planning time [ms]')
ylabel('Number of visited nodes')
legend('Breadth First','Dijkstra','A*','BestFirst','Location','SouthEast')
title('Mission one')

figure(12)
plot(bf_plan.time.*1000,bf_plan.length,'*', dijkstra_plan.time.*1000,dijkstra_plan.length,'*', ...
    astar_plan.time.*1000,astar_plan.length,'*',bestfirst_plan.time.*1000,bestfirst_plan.length,'*')
xlabel('Planning time [ms]')
ylabel('Plan length [m]')
legend('Breadth First','Dijkstra','A*','BestFirst')
title('Mission one')

figure(13)
plot(dijkstra_plan.time.*1000,dijkstra_plan.length,'*',astar_plan.time.*1000,astar_plan.length,'*')
xlabel('Planning time [ms]')
ylabel('Plan length [m]')
legend('Dijkstra','A*')
title('Mission one')



%%
dijkstra_plan = Dijkstra(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    dijkstra_plan.length, dijkstra_plan.num_visited_nodes, dijkstra_plan.time*1000);
%%
dist_to_go=zeros(num_nodes,1);
for i = 1:num_nodes
    dist_to_go(i)=b2_cost_to_go(num_nodes, i, mission.goal.id, f_next);
end
