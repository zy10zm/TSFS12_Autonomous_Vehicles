function y = h5(x, measpar)
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
y=[];
for i = 1:size(measpar.idx, 3)
    y = [y; x(measpar.idx(2, :, i)) - x(measpar.idx(1, :, i))];
end

end
