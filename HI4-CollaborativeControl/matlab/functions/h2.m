
function y = h2(x, measpar)
% Measurement function
%
% The measurement function gives the measurements available to the agent.
%
% Input arguments:
%   x - the complete state vector of all agents in the multi-agent system
%   measpar - structure containing parameters used in the function. In
%             this case the indices in the state vector x of the states
%             measured by the agent stored in meas_idx.
%
% Output:
%   y - The measurement vector

y = x(measpar.idx);
end