close all
clear all

addpath functions

% Specify which dynamic model that is used
agent(1).f = @f1;
agent(2).f = @f1;
agent(3).f = @f1;
agent(4).f = @f1;
agent(5).f = @f1;
agent(6).f = @f1;
agent(7).f = @f1;
agent(8).f = @f1;
% The dynamic model f1 is as single integrator model.

% The model parameters is the mass, which is not used in this example, and
% the wind disturbance.
modelparam.m = 1;
modelparam.w = [0; 0];

% Specify which set of parameters that is used in the dynamic model.
agent(1).mdlpar = modelparam;
agent(2).mdlpar = modelparam;
agent(3).mdlpar = modelparam;
agent(4).mdlpar = modelparam;
agent(5).mdlpar = modelparam;
agent(6).mdlpar = modelparam;
agent(7).mdlpar = modelparam;
agent(8).mdlpar = modelparam;


% Specify which controller model that is used
agent(1).g = @g1;
agent(2).g = @g5;
agent(3).g = @g5;
agent(4).g = @g5;
agent(5).g = @g5;
agent(6).g = @g5;
agent(7).g = @g5;
agent(8).g = @g5;


% Two different proportionality constants are used by the controller in
% this example.
controlparam1.k_v = 1;
controlparam1.k_p = 1;
controlparam2.k_v = 1;
controlparam2.k_p = 1;

% Specify which set of parameters that is used by the controller
agent(1).ctrlpar = controlparam1;
agent(2).ctrlpar = controlparam2;
agent(3).ctrlpar = controlparam2;
agent(4).ctrlpar = controlparam2;
agent(5).ctrlpar = controlparam2;
agent(6).ctrlpar = controlparam2;
agent(7).ctrlpar = controlparam2;
agent(8).ctrlpar = controlparam2;


% Specify which measure model that is used
agent(1).h = @h2;
agent(2).h = @h5;
agent(3).h = @h5;
agent(4).h = @h5;
agent(5).h = @h5;
agent(6).h = @h5;
agent(7).h = @h5;
agent(8).h = @h5;


% Specify index in the state vector of the measured signals
idx = {1:2, 3:4, 5:6,7:8, 9:10, 11:12, 13:14, 15:16};

agent(1).measpar.idx = idx{1};
agent(2).measpar.idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}],...
                                  [idx{2};idx{4}],[idx{2};idx{5}]);
agent(3).measpar.idx = cat(3,[idx{3};idx{1}],[idx{3};idx{2}],...
                                  [idx{3};idx{5}]);
agent(4).measpar.idx = cat(3,[idx{4};idx{2}],[idx{4};idx{5}],...
                                  [idx{4};idx{6}]);
agent(5).measpar.idx = cat(3,[idx{5};idx{2}],[idx{5};idx{3}],...
                                  [idx{5};idx{4}],[idx{5};idx{6}],...
                                  [idx{5};idx{7}]);
agent(6).measpar.idx = cat(3,[idx{6};idx{4}],[idx{6};idx{5}],...
                                  [idx{6};idx{7}],[idx{6};idx{8}]);
agent(7).measpar.idx = cat(3,[idx{7};idx{5}],[idx{7};idx{6}],...
                                  [idx{7};idx{8}]);
agent(8).measpar.idx = cat(3,[idx{8};idx{6}],[idx{8};idx{7}]);

% Reference position for the agents

w=0.5;
agent(1).xref = @(t) [0 6]'+2*[cos(w*t) sin(w*t)]'; % Global reference
agent(2).xref = @(t) [1 1;2 0;0 -2;2 -2]'; % Relative references
agent(3).xref = @(t) [-1 1;-2 0;0 -2]';
agent(4).xref = @(t) [0 2;2 0;0 -2]';
agent(5).xref = @(t) [-2 2;0 2;-2 0;-2 -2;0 -2]';
agent(6).xref = @(t) [0 2;2 2;2 0;1 -1]';
agent(7).xref = @(t) [0 2;-2 0;-1 -1]';
agent(8).xref = @(t) [-1 1;1 1]';

% SIMULATION

n=2; % Number of states

x0 = [1 0, 2 0, 3 0, 4 0,...
     5 0, 6 0, 7 0, 8 0]'; 

% x0 = [1 0, 2 0, 3 0, 4 0,...
%     1 3 -1 1 1 1 0 0]'; % These initial values do not work. This implies
%     that the graph is NOT globally rigid.

tspan = 0:0.05:40; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
figure(1)
title('Position based control of four agents.')
xlabel('m')
ylabel('m')

for i=1:length(t)
    plot(x(i,1),x(i,2),'rx',x(i,3),x(i,4),'bx',x(i,5),x(i,6),'bx',x(i,7),x(i,8),'bx',...
        x(i,9),x(i,10),'bx',x(i,11),x(i,12),'bx',x(i,13),x(i,14),'bx',x(i,15),x(i,16),'bx')
    axis([-15 15 -15 15])
    pause(0.03)
end


 %% BOTTOM AGENT CAN MEASURE ABSOLUTE POSITION

% Specify index in the state vector of the measured signals
idx = {1:2, 3:4, 5:6,7:8, 9:10, 11:12, 13:14, 15:16};

agent(1).measpar.meas_idx = idx{1};
agent(2).measpar.meas_idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}],...
                                  [idx{2};idx{4}],[idx{2};idx{5}]);
agent(3).measpar.meas_idx = cat(3,[idx{3};idx{1}],[idx{3};idx{2}],...
                                  [idx{3};idx{5}]);
agent(4).measpar.meas_idx = cat(3,[idx{4};idx{2}],[idx{4};idx{5}],...
                                  [idx{4};idx{6}]);
agent(5).measpar.meas_idx = cat(3,[idx{5};idx{2}],[idx{5};idx{3}],...
                                  [idx{5};idx{4}],[idx{5};idx{6}],...
                                  [idx{5};idx{7}]);
agent(6).measpar.meas_idx = cat(3,[idx{6};idx{4}],[idx{6};idx{5}],...
                                  [idx{6};idx{7}],[idx{6};idx{8}]);
agent(7).measpar.meas_idx = cat(3,[idx{7};idx{5}],[idx{7};idx{6}],...
                                  [idx{7};idx{8}]);
agent(8).measpar.meas_idx = idx{8};

% Reference position for the agents
w=0.5; % Angular frequency
agent(1).xref = @(t) [0 6]'+2*[cos(w*t) sin(w*t)]'; % Global reference
agent(2).xref = @(t) [1 1;2 0;0 -2;2 -2]'; % Relative references
agent(3).xref = @(t) [-1 1;-2 0;0 -2]';
agent(4).xref = @(t) [0 2;2 0;0 -2]';
agent(5).xref = @(t) [-2 2;0 2;-2 0;-2 -2;0 -2]';
agent(6).xref = @(t) [0 2;2 2;2 0;1 -1]';
agent(7).xref = @(t) [0 2;-2 0;-1 -1]';
agent(8).xref = @(t) [0 0]'+2*[cos(w*t) sin(w*t)]'; % Global reference

n=2; % Number of states

%x0 = [1 0, 2 0, 3 0, 4 0,...
    % 5 0, 6 0, 7 0, 8 0]'; 
x0 = [1 0, 2 0, 3 0, 4 0,...
     1 3 -1 1 1 1 0 0]'; % Now this initial state that didn't work before actually works-

tspan = 0:0.05:40; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
figure(1)
title('Distance based control of four agents.')
xlabel('m')
ylabel('m')

for i=1:length(t)
    plot(x(i,1),x(i,2),'rx',x(i,3),x(i,4),'bx',x(i,5),x(i,6),'bx',x(i,7),x(i,8),'bx',...
        x(i,9),x(i,10),'bx',x(i,11),x(i,12),'bx',x(i,13),x(i,14),'bx',x(i,15),x(i,16),'bx')
    axis([-15 15 -15 15])
    pause(0.03)
end
