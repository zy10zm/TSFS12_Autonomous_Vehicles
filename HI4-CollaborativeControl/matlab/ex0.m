close all
clear all

addpath functions


% THE AGENTS IN THE MULTI-AGENT SYSTEM

% Define the structure 'agent'. that contains information about the four
% agents in this exampel. The functions f1, h1, and g1, used below, can be
% founnd in the directory 'functions'.

% Specify which dynamic model that is used
agent(1).f = @f1;
agent(2).f = @f1;
agent(3).f = @f1;
agent(4).f = @f1;
% The dynamic model f1 is as single integrator model. You control the
% velocity of the agent. The acceleration of (or force on) the agent will
% be controlled in later assignments.

% The model parameter is the mass, which is not used in this example
modelparam.m = 1;

% Specify which set of parameters that is used in the dynamic model.
agent(1).mdlpar = modelparam;
agent(2).mdlpar = modelparam;
agent(3).mdlpar = modelparam;
agent(4).mdlpar = modelparam;

% Specify which controller model that is used
agent(1).g = @g1;
agent(2).g = @g1;
agent(3).g = @g1;
agent(4).g = @g1;
% The controllerT g1 is a P-controller, which gives a control signal
% proportional to the vector between the current position and the reference
% position. A PD-controller will be used later.


% Two different proportionality constants are used by the controller in
% this example.
controlparam1.k = 1;
controlparam2.k = 10;

% Specify which set of parameters that is used by the controller
agent(1).ctrlpar = controlparam2;
agent(2).ctrlpar = controlparam1;
agent(3).ctrlpar = controlparam1;
agent(4).ctrlpar = controlparam2;

% Specify which measure model that is used
agent(1).h = @h1;
agent(2).h = @h1;
agent(3).h = @h1;
agent(4).h = @h1;
% The measurment h1 gives the absolute position in a global coordinate
% system

% Specify index in the state vector of the measured signals
agent(1).measpar.meas_idx = 1:2;
agent(2).measpar.meas_idx = 3:4;
agent(3).measpar.meas_idx = 5:6;
agent(4).measpar.meas_idx = 7:8;

% Reference position for the agents
agent(1).xref = @(t) [cos(2*t) sin(2*t)]'; % Moving counter-clockwise on a circle 1
agent(2).xref = @(t) [2*cos(-t) 2*sin(-t)]'; % Moving clockwaise on a circle with radius 2
agent(3).xref = @(t) [3 2]'; % Fixed position
agent(4).xref = @(t) [5 4]'; % Fixed position


% SIMULATION

n=2; %number of states for each agent

x0 = [1 1 2 0 3 0 4 0]'; % Initial values of the state vector.
tspan = 0:0.05:10; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
for i=1:length(t)
    plot(x(i,1),x(i,2),'x',x(i,3),x(i,4),'o',x(i,5),x(i,6),'s',x(i,7),x(i,8),'d')
    axis([-2 6 -2 6])
    pause(0.03)
end


