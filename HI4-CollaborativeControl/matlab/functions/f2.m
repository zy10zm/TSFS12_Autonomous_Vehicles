
function dxdt = f2(t, x, u, mdlpar)
% Dynamic function for single integrator model
%     
% Input arguments are:
%   t - time 
%   x - state vector of the agent (position) 
%   u - the control input u, 
%   mdlpar - structure with parameters of the model 
%     
% Output:
%   dxdt - state-derivative

dxdt = [x(3:4); u + mdlpar.w];
end
