clear
addpath Functions

addpath("casadi-windows-matlabR2016a-v3.5.5") 

%% Simulate a simple path to follow
car = SingleTrackModel();
car.controller = MiniController();
w0 = [0, 0, 0, 2];
[~, w, ~] = car.simulate(w0, 40, 0.1);
fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
ref_path = SplinePath(p);

s = linspace(0, ref_path.length, 200);

% Plot resulting path
figure(10)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(p(:, 1), p(:, 2), 'rx');
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');
box off

%% Run the MPC path following controller
% Implement an MPC controller in the ModelPredictiveController skeleton class.
% Parameters for the controller are
%
% gamma_d - Weight in the loss-function for _distance errors_
% gamma_theta - Weight in the loss-function for _heading errors_
% gamma_u - Weight in the loss-function for _control signal_ (steer angle)
% L - wheel base
% steer_limit - Steer limits (in radians) for the control signal

opts.h_p = 10;
opts.gamma_d = 1;
opts.gamma_theta = 1;
opts.gamma_u = 50;
opts.L = car.L;
opts.steer_limit = pi / 4;

car = SingleTrackModel();
car.steer_limit = opts.steer_limit;

mpc = ModelPredictiveController(opts, ref_path, 0.1);
car.controller = mpc;

w0 = [0, 6, 0.9*pi/2, 2];
[t, w, u] =  car.simulate(w0, 300, mpc.sample_rate, 0.0);
z_mpc = {t, w, u};
fprintf('Total time in controller: %.2f sek\n', mpc.u_time);

M = 1;
mpc_path = w(1:M:end,1:2);
mpc_splinepath = SplinePath(mpc_path);



% Plot resulting paths and control signals
figure(14)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(mpc_path(:,1),mpc_path(:,2), 'r')
hold off
title(['Path from MPC, gamma-d = ' num2str(opts.gamma_d) ', gamma-theta = ' ...
    num2str(opts.gamma_theta) ', gamma-u = ' num2str(opts.gamma_u)]);
xlabel('x [m]');
ylabel('y [m]');
legend('Reference path', 'Path from MPC')

control_limit_plotvec = opts.steer_limit*ones(1,numel(t));



%%
figure(15)
p(1) = plot(t, u(:,1),'b','LineWidth',2);
hold on
p(2) = plot(t, -control_limit_plotvec, 'r','LineWidth',2);
plot(t, control_limit_plotvec, 'r','LineWidth',2);
title('Control signal from MPC')
legend([p(1),p(2)],'Control signal','Bounds of control signal')
ylim([-1 1])