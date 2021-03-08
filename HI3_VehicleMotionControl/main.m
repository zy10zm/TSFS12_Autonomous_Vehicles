clear
addpath Functions

%% Define a simple vehicle controller, simulate, and create a reference path
car = SingleTrackModel();  % Create car object
car.controller = MiniController();  % Create controller object and assign to car
w0 = [0, 0, 0, 2];  % Initial state (x, y, theta, v)
[t, w, u] = car.simulate(w0, 40, 0.1);  % Simulate closed-loop system

z_simple = {t, w, u};  % Save results
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
ref_path = SplinePath(p); %% Create path from MiniController output

%% Pure pursuit controller
% Create vehicle and controller objects
%ref_path = SplinePath(p); %% Create path from MiniController output

car = SingleTrackModel();
pp_controller = PurePursuitController(6, car.L, ref_path,0.5);
car.controller = pp_controller;

% YOUR CODE HERE

w0 = [0, 0, 0, 2];  % Initial state (x, y, theta, v)

[t, w, u] = car.simulate(w0, 50, 0.1);  % Simulate closed-loop system

z_simple = {t, w, u};  % Save results
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);

pure_path = SplinePath(p);
s = linspace(0, ref_path.length, 200);

% Plot resulting paths and control signals
figure(12)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(pure_path.x(s), pure_path.y(s), 'r')
%plot(p(:, 1), p(:,2), 'rx');
hold off
title('Path from Pure Pursuit controller');
xlabel('x [m]');
ylabel('y [m]');
legend('Reference path', 'Path from Pure Pursuit')

figure(13)
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

subplot(1, 2, 2)
plot(t, u(:,2))
xlabel('t [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration');
box off

%% LQR Path tracker with linear feedback
p = [0 0; 1 0; 2 0; 3 0; 4 0; 5 0; 6 0; 7 0; 8 0;];
ref_path = SplinePath(p);

v = 2; 
Ts = 0.1;
sys = ss([0,v;0,0],[0;v],[1,1],0);
sys_d=c2d(sys,0.1);
Q =eye(2);
R = 1;
S = [];
E = sys_d.E;

[X, K, L] = idare(sys_d.A,sys_d.B,Q,R,S,E);
car=SingleTrackModel();
lf_controller = StateFeedbackController(K,car.L,ref_path,0.5);
car.controller = lf_controller;
Tend = 80;
w0 = [-2,0.5,0,2];
%w0 = [-5, 10, pi/4, 2];)
[t,w,u] = car.simulate(w0,Tend,Ts);
M = 1;
state_feedback_path = w(1:M:end,1:2);
pl_stf = SplinePath(state_feedback_path);

%z_simple = {t, w, u};  % Save results
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
s = linspace(0, ref_path.length, 200);
% Plot resulting paths and control signals
figure(14)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(pl_stf.x(s), pl_stf.y(s), 'r')
hold off
title('Path from Linear State Feedback');
xlabel('x [m]');
ylabel('y [m]');
legend('Reference path', 'Path from Linear State Feedback')

figure(15)
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

subplot(1, 2, 2)
plot(t, u(:,2))
xlabel('t [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration');
box off

%% LQR Path tracker with non-linear feedback

% YOUR CODE HERE
v = 2; 
Ts = 0.1;

K=[1,4]; %Guessed tuning parameters
car=SingleTrackModel();
nlf_controller = NonLinearStateFeedbackController(K,car.L,ref_path,0.2);
car.controller = nlf_controller;

Tend = 80;
w0 = [-5, 10, 0, 2];
%w0 = [-5, 10, 0.9*pi/2, 2];
[t,w,u] = car.simulate(w0,Tend,Ts);
M = 10;
state_feedback_path = w(1:M:end,1:2);
pl_nstf = SplinePath(state_feedback_path);

z_simple = {t, w, u};  % Save results
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);

% Plot resulting paths and control signals
figure(16)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(pl_nstf.x(s), pl_nstf.y(s), 'r')
hold off
title('Path from non linear state feedback');
xlabel('x [m]');
ylabel('y [m]');
legend('Reference path', 'Path from non linear state feedback')

figure(17)
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

subplot(1, 2, 2)
plot(t, u(:,2))
xlabel('t [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration');
box off


