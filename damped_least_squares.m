function [dq, achieved] = damped_least_squares(J, v, lambda)
%DAMPED_LEAST_SQUARES  Damped least squares solution of J * dq = v.
%
%   DQ = damped_least_squares (J, V, LAMBDA) returns the joint velocity that
%   best realises the task velocity V, using the Levenberg-Marquardt damped
%   pseudo inverse
%
%       DQ = J' * (J * J' + LAMBDA^2 * I)^-1 * V
%
%   The damping LAMBDA keeps the solution finite and smooth near kinematic
%   singularities, where a plain PSEUDOINVERSE would produce very large or
%   discontinuous joint velocities.  Use a small LAMBDA (or zero for a square,
%   well conditioned J) when accuracy matters more than smoothness.
%
%   [DQ, ACHIEVED] = damped_least_squares (...) also returns the task velocity
%   ACHIEVED = J * DQ that the solution actually produces.
%
%   See also AVOIDANCE_CONTROLLER.

if nargin < 3 || isempty(lambda)
    lambda = 0.01;
end

J = squeeze(J);
v = v(:);
m = size(J, 1);

if size(J, 2) == 1
    % A single row of unknowns: (J'*J + l^2) is a scalar, handle it directly.
    Jv = J(:, 1);
    dq = Jv * (v.' * Jv) / (Jv.' * Jv + lambda^2);
    dq = dq(:);
elseif lambda > 0
    dq = J.' * ((J * J.' + lambda^2 * eye(m)) \ v);
else
    dq = pinv(J) * v;
end

if any(~isfinite(dq))
    dq = pinv(J) * v;
end

if nargout > 1
    achieved = J * dq;
end

end

%!test
%! % a full rank square system is solved exactly when the damping is zero
%! A = [2 1; 1 3];
%! b = [1; 2];
%! dq = damped_least_squares(A, b, 0);
%! assert(A * dq, b, 1e-9);

%!test
%! % damping keeps the solution bounded at a singularity
%! A = [1 0; 0 1; 0 0];
%! [dq, ach] = damped_least_squares(A, [1; 1; 1], 0.5);
%! assert(norm(dq) < 2);
%! assert(ach, [1; 1; 0] * (1 / (1 + 0.25)), 1e-9);

%!test
%! % without damping a singular direction is simply dropped
%! A = [1 0; 0 1; 0 0];
%! dq = damped_least_squares(A, [1; 1; 1], 0);
%! assert(dq, [1; 1], 1e-9);
