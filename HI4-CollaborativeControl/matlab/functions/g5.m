function u = g5(y, xref, ctrlpar)
% Control function
%
% Compute the control signal, in this case it is a gradient control
%
% Input arguments:
%   y - measurement y,
%   xref - the reference vector xref (in this case the desired position
%           of the agent)
%   ctrlpar - dictionary which contains parameters used by the controller
%             (in this case the proportional gain kp).
%
% Output argument:
%   Control signal

y = reshape(y, 2, []);

u=[0;0];
for j = 1:size(xref, 2)
    gammaij_prime = (norm(y(:, j))^2 - norm(xref(1:2, j))^2)*4*norm(y(:, j));
    u = u + ctrlpar.k_p*gammaij_prime*y(:, j)/(norm(y(:, j)));
end

end
