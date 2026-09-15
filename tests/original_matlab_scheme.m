function [pos, axes_world] = original_matlab_scheme(theta_step, rounds, model, axis_mode)
%ORIGINAL_MATLAB_SCHEME  Literal copy of the kinematics of the original demo.
%
%   This reproduces, step by step, the incremental axis rotation scheme of the
%   original MATLAB obstacle_avoidance.m: three basis vector matrices are
%   rotated in place once per time step and the link vectors are accumulated
%   with CUMSUM.  The original applied the same DTHETA at every step, so after
%   ROUNDS steps the arm is at joint angles ROUNDS * THETA_STEP.
%
%   AXIS_MODE selects which rotation axis each joint uses inside a round:
%
%     'frozen'  (default) the literal behaviour of the original script, which
%               read the seven axes once at the top of the round and then
%               applied all seven rotations with those stale axes.  Because
%               joint 1 has already moved joint 2's axis by the time joint 2
%               rotates, this is a first order approximation: it converges to
%               the exact kinematics as the step shrinks, with an O(step)
%               error for a finite step.
%
%     'updated' the axis of joint i is re-read after joints 1..i-1 have moved,
%               which is the exact incremental form and matches the closed
%               form kinematics in ROBOT_KINEMATICS to machine precision.
%
%   The helper exists only so that the ported kinematics can be checked against
%   the code it replaces.  It is not used by the simulation.
%
%   See also ROBOT_KINEMATICS, TEST_MODEL_FIDELITY.

if nargin < 4 || isempty(axis_mode)
    axis_mode = 'frozen';
end

n = model.n;
if n ~= 7
    error('original_matlab_scheme:unsupportedModel', ...
        'The original scheme is hard coded for the 7-DOF arm.');
end

theta_step = theta_step(:).';

x = repmat([1; 0; 0], 1, n);
y = repmat([0; 1; 0], 1, n);
z = repmat([0; 0; 1], 1, n);

for step = 1:rounds
    if ~strcmpi(axis_mode, 'updated')
        % The original read the seven axes once per round and then applied all
        % seven rotations with those stale axes.
        axes_world = [z(:, 1), y(:, 2), x(:, 3), y(:, 4), z(:, 5), y(:, 6), x(:, 7)];
    end
    for i = 1:n
        if strcmpi(axis_mode, 'updated')
            axes_world = [z(:, 1), y(:, 2), x(:, 3), y(:, 4), z(:, 5), y(:, 6), x(:, 7)];
        end
        R = RotateBy(axes_world(:, i), theta_step(i));
        x(:, i:n) = R * x(:, i:n);
        y(:, i:n) = R * y(:, i:n);
        z(:, i:n) = R * z(:, i:n);
    end
end

axes_world = [z(:, 1), y(:, 2), x(:, 3), y(:, 4), z(:, 5), y(:, 6), x(:, 7)];
link_vectors = [ ...
    model.link_lengths(1) * z(:, 1), ...
    model.link_lengths(2) * x(:, 2), ...
    model.link_lengths(3) * x(:, 3), ...
    model.link_lengths(4) * z(:, 4), ...
    model.link_lengths(5) * z(:, 5), ...
    model.link_lengths(6) * x(:, 6), ...
    zeros(3, 1)];

pos = cumsum([model.base_offset, link_vectors], 2);

end
