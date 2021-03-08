close all
clear all

addpath functions


% Specify which dynamic model that is used

agent(1).f = @f2;
agent(2).f = @f2;
agent(3).f = @f2;
agent(4).f = @f2;
agent(5).f = @f2;
agent(6).f = @f2;
agent(7).f = @f2;
agent(8).f = @f2;
% The dynamic model f2 is as double integrator model.

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
agent(1).g = @g2;
agent(2).g = @g4;
agent(3).g = @g4;
agent(4).g = @g4;
agent(5).g = @g4;
agent(6).g = @g4;
agent(7).g = @g4;
agent(8).g = @g4;

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
agent(2).h = @h4;
agent(3).h = @h4;
agent(4).h = @h4;
agent(5).h = @h4;
agent(6).h = @h4;
agent(7).h = @h4;
agent(8).h = @h4;


% Specify index in the state vector of the measured signals
idx = {1:4, 5:8, 9:12, 13:16, 17:20, 21:24, 25:28, 29:32};

agent(1).measpar.idx = idx{1};
agent(2).measpar.idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}]);
agent(3).measpar.idx = cat(3,[idx{3};idx{2}],[idx{3};idx{4}]);
agent(4).measpar.idx = cat(3,[idx{4};idx{3}],[idx{4};idx{5}]);
agent(5).measpar.idx = cat(3,[idx{5};idx{4}],[idx{5};idx{6}]);
agent(6).measpar.idx = cat(3,[idx{6};idx{5}],[idx{6};idx{7}]);
agent(7).measpar.idx = cat(3,[idx{7};idx{6}],[idx{7};idx{8}]);
agent(8).measpar.idx = cat(3,[idx{8};idx{7}]);

% Reference position for the agents
agent(1).xref = @(t) [0 4 0 0]'; % Global reference
agent(2).xref = @(t) [0 0;1 -1; 0 0;0 0]; % Relative references
agent(3).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(4).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(5).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(6).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(7).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(8).xref = @(t) [0 1 0 0]';

% SIMULATION

% Number of states for each agent
n=4;

x0 = [1 0 0 0 2 0 0 0 3 0 0 0 4 0 0 0 ...
    5 0 0 0 6 0 0 0 7 0 0 0 8 0 0 0]'; % Initial values of the state vector.
tspan = 0:0.05:1000; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
figure(1)
title('Position based control of four agents.')
xlabel('m')
ylabel('m')
for i=1:length(t)
    plot(x(i,1),x(i,2),'bx',x(i,5),x(i,6),'bx',x(i,9),x(i,10),'bx',x(i,13),x(i,14),'bx',...
        x(i,17),x(i,18),'bx',x(i,21),x(i,22),'bx',x(i,25),x(i,26),'bx',x(i,29),x(i,30),'bx')
    axis([-15 15 -15 15])
    pause(0.03)
end

% Plotting final positions

figure(2)
plot(x(end,1),x(end,2),'bx',x(end,5),x(end,6),'bx',x(end,9),x(end,10),'bx',x(end,13),x(end,14),'bx',...
        x(end,17),x(end,18),'bx',x(end,21),x(end,22),'bx',x(end,25),x(end,26),'bx',x(end,29),x(end,30),'bx')
    axis([-15 15 -15 15])

    
    
 %% Oscillator
 
 % Specify index in the state vector of the measured signals
 
idx = {1:4, 5:8, 9:12, 13:16, 17:20, 21:24, 25:28, 29:32};
 
agent(1).measpar.idx = idx{1};
agent(2).measpar.idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}]);
agent(3).measpar.idx = [idx{3};idx{2}];
agent(4).measpar.idx = [idx{4};idx{3}];
agent(5).measpar.idx = [idx{5};idx{4}];
agent(6).measpar.idx = [idx{6};idx{5}];
agent(7).measpar.idx = [idx{7};idx{6}];
agent(8).measpar.idx = [idx{8};idx{7}];

% Reference position for the agents
agent(1).xref = @(t) [0 sin(5*t) 0 0]';
agent(2).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(3).xref = @(t) [0; 1; 0; 0];
agent(4).xref = @(t) [0; 1; 0; 0];
agent(5).xref = @(t) [0; 1; 0; 0];
agent(6).xref = @(t) [0; 1; 0; 0];
agent(7).xref = @(t) [0; 1; 0; 0];
agent(8).xref = @(t) [0 1 0 0]';

% Two different proportionality constants are used by the controller in
% this example.
controlparam1.k_v = 5;
controlparam1.k_p = 5;
controlparam2.k_v = 30;
controlparam2.k_p = 30;

% Specify which set of parameters that is used by the controller
agent(1).ctrlpar = controlparam1;
agent(2).ctrlpar = controlparam2;
agent(3).ctrlpar = controlparam2;
agent(4).ctrlpar = controlparam2;
agent(5).ctrlpar = controlparam2;
agent(6).ctrlpar = controlparam2;
agent(7).ctrlpar = controlparam2;
agent(8).ctrlpar = controlparam2;

% SIMULATION

% Number of states for each agent
n=4;


x0 = [0 0 0 0 0 -1 0 0 0 -2 0 0 0 -3 0 0 0 ...
    -4 0 0 0 -5 0 0 0 -6 0 0 0 -7 0 0]'; % Initial values of the state vector.
tspan = 0:0.05:40; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
figure(1)
title('Position based control of four agents.')
xlabel('m')
ylabel('m')
for i=1:length(t)
    plot(x(i,1),x(i,2),'bx',x(i,5),x(i,6),'bx',x(i,9),x(i,10),'bx',x(i,13),x(i,14),'bx',...
        x(i,17),x(i,18),'bx',x(i,21),x(i,22),'bx',x(i,25),x(i,26),'bx',x(i,29),x(i,30),'bx')
    axis([-3 3 -3 3])
    pause(0.03)
end


% COMMENT: With a lower angular angular frequency, the agents only
% measuring relative position and velocity follow the first agent good. And
% they are in phase with one another. With a higher angular frequency, the
% agents i=2,...,8 barely oscillate, this probably due to that they dont
% have the time to react to the fast oscillation of the first agent. This
% is also a chain reaction. Since agent 2 barely oscillate, the others down
% in the list will oscillate even less.
% But by increasing the k_p and k_v values for agents i=2,...,8 such that
% they are quite a bit larger than the control parameters for the first
% agent, the agents i=2,...,8 are able to oscillate in sync with the first
% agent quite well. E.g. if agent 1 follows the reference sin(5*t), and
% have the control parameters k_p=k_v=5, the control parameters k_p=k_v=30
% works quite well for the agents i = 2,...,8.
