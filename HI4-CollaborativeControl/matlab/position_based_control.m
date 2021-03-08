close all
clear all

addpath functions


% Specify which dynamic model that is used
agent(1).f = @f2;
agent(2).f = @f2;
agent(3).f = @f2;
agent(4).f = @f2;
% The dynamic model f2 is as double integrator model.

% The model parameters is the mass, which is not used in this example, and
% the wind disturbance.
modelparam.m = 1;
modelparam.w = [0.2; 0]; % wind in positive x-dir.  

% Specify which set of parameters that is used in the dynamic model.
agent(1).mdlpar = modelparam;
agent(2).mdlpar = modelparam;
agent(3).mdlpar = modelparam;
agent(4).mdlpar = modelparam;

% Specify which controller model that is used
agent(1).g = @g2;
agent(2).g = @g2;
agent(3).g = @g2;
agent(4).g = @g2;

% Two different proportionality constants are used by the controller in
% this example.
controlparam1.k_v = 1;
controlparam1.k_p = 1;
controlparam2.k_v = 1;
controlparam2.k_p = 1;

% Specify which set of parameters that is used by the controller
agent(1).ctrlpar = controlparam2;
agent(2).ctrlpar = controlparam1;
agent(3).ctrlpar = controlparam1;
agent(4).ctrlpar = controlparam2;

% Specify which measure model that is used
agent(1).h = @h2;
agent(2).h = @h2;
agent(3).h = @h2;
agent(4).h = @h2;

% Specify index in the state vector of the measured signals
agent(1).measpar.idx = 1:4;
agent(2).measpar.idx = 5:8;
agent(3).measpar.idx = 9:12;
agent(4).measpar.idx = 13:16;

% Reference position for the agents
agent(1).xref = @(t) [cos(t) sin(t) -modelparam.w(1) 0]'; % Moving counter-clockwise on a circle 1
agent(2).xref = @(t) [2*cos(-t) 2*sin(-t) -modelparam.w(1) 0]'; % Moving clockwaise on a circle with radius 2
agent(3).xref = @(t) [3*cos(t) 3*sin(2*t) -modelparam.w(1) 0]'; % Infinity curve
agent(4).xref = @(t) [4 4 -modelparam.w(1) 0]'; % Fixed position


% SIMULATION

% Number of states for each agent
n=4; 

x0 = [1 1 0 0 2 0 0 0 0 0 0 0 4 0 0 0]'; % Initial values of the state vector.
tspan = 0:0.05:20; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
figure(1)
title('Position based control of four agents.')
xlabel('m')
ylabel('m')
for i=1:length(t)
    plot(x(i,1),x(i,2),'bx',x(i,5),x(i,6),'ko',x(i,9),x(i,10),'rs',x(i,13),x(i,14),'md')
    axis([-6 6 -6 6])
    pause(0.03)
end

