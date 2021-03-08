
function u = g1(y, xref, ctrlpar)
% Control function
%     
% Compute the control signal, in this case it is a P-controller which gives
% a control signal proportional to the vector between the current position
% and the reference position. 
%  
% Input arguments:
%   y - measurement y, 
%   xref - the reference vector xref (in this case the desired position of
%          the agent)
%   ctrlpar - dictionary which contains parameters used by the controller 
%             (in this case the proportional gain k).
% 
% Output argument:
%   Control signal 


u = ctrlpar.k_p*(xref-y);
end
