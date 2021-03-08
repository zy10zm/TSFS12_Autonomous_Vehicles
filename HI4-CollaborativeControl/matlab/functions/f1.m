
function dxdt = f1(t, x, u, mdlpar)
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

dxdt = u;
end
