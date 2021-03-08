% This script displays the minimally rigid graph used in the distance based
% formation control.

t = [1 1 2 2 2 3 4 4 5 5 6 6 7];
h = [2 3 3 4 5 5 5 6 6 7 7 8 8]; 
pos = [0 6; -1 5; 1 5; -1 3; 1 3; -1 1; 1 1; 0 0]; % coordinates of nodes 1:8
g = graph(t,h);
figure(1)
plot(g, 'XData', pos(:,1), 'YData', pos(:,2));