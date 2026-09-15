function [pos, axes_world, frames] = robot_kinematics(theta, model)
%ROBOT_KINEMATICS  Forward kinematics of the demo manipulator.
%
%   [POS, AXES] = robot_kinematics (THETA, MODEL) evaluates the arm at joint
%   angles THETA (1xN or Nx1, radians).
%
%   POS is 3x(N+1): POS(:,1) is the base of the first link, POS(:,k+1) is the
%   end of link k, i.e. the end effector for k = N.
%
%   AXES is 3xN: AXES(:,k) is the world frame direction of joint k's rotation
%   axis at this configuration.  It is also the direction of link k, because
%   every link of this arm extends along its own joint axis.
%
%   [POS, AXES, FRAMES] = robot_kinematics (...) also returns the cumulative
%   rotations, 3x3x(N+1), with FRAMES(:,:,k) = C{k-1}, the frame in which
%   joint k's local axis is expressed.  FRAMES(:,:,N+1) = C{N} is the frame
%   of the last link and is used for drawing the gripper.
%
%   See also PA10_MODEL, ROBOT_JACOBIAN.

theta = theta(:).';
n = model.n;
if numel(theta) ~= n
    error('robot_kinematics:badTheta', ...
        'THETA must have %d elements, got %d.', n, numel(theta));
end

pos = zeros(3, n + 1);
axes_world = zeros(3, n);
if nargout > 2
    frames = zeros(3, 3, n + 1);
end

C = eye(3);
pos(:, 1) = model.base_offset;
if nargout > 2
    frames(:, :, 1) = C;
end

for k = 1:n
    joint_axis = model.joint_axes(:, k);
    axes_world(:, k) = C * joint_axis;
    C = C * RotateBy(joint_axis, theta(k));
    pos(:, k + 1) = pos(:, k) + model.link_lengths(k) * (C * model.link_axes(:, k));
    if nargout > 2
        frames(:, :, k + 1) = C;
    end
end

end

%!test
%! model = pa10_model();
%! [pos, axes] = robot_kinematics(zeros(1, 7), model);
%! assert(pos(:, 1), [0; 0; 0.1], 1e-12);
%! assert(pos(:, 2), [0; 0; 0.1 + 0.1135], 1e-12);
%! assert(pos(:, 3), [0.307; 0; 0.1 + 0.1135], 1e-12);
%! assert(pos(:, 4), [0.307 + 0.143; 0; 0.1 + 0.1135], 1e-12);
%! assert(pos(:, 8), pos(:, 7), 1e-12);   % link 7 has zero length
%! assert(axes(:, 1), [0; 0; 1], 1e-12);
%! assert(axes(:, 2), [0; 1; 0], 1e-12);

%!test
%! % joint 1 is a base rotation, so it must not move links 1 and 2
%! model = pa10_model();
%! [p0, ~] = robot_kinematics(zeros(1, 7), model);
%! [p1, ~] = robot_kinematics([pi/3, 0, 0, 0, 0, 0, 0], model);
%! assert(p1(:, 1), p0(:, 1), 1e-12);
%! assert(p1(:, 2), p0(:, 2), 1e-12);
%! assert(norm(p1(:, 3) - p0(:, 3)) > 0.1);

%!test
%! % the frame output must agree with the axis output
%! model = pa10_model();
%! theta = [0.3, -0.4, 0.5, -0.2, 0.6, 0.1, -0.3];
%! [~, axes, frames] = robot_kinematics(theta, model);
%! for k = 1:model.n
%!     assert(axes(:, k), frames(:, :, k) * model.joint_axes(:, k), 1e-12);
%! end
