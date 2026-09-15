function [J, pos, axes_world] = robot_jacobian(theta, model, link_index, point)
%ROBOT_JACOBIAN  Geometric Jacobian of the demo manipulator, base frame.
%
%   J = robot_jacobian (THETA, MODEL) evaluates the 6xN geometric Jacobian of
%   the tool centre point, i.e. the end of the last link.  The first three
%   rows map joint velocities to the linear velocity of that point, the last
%   three rows map them to the angular velocity of the last link, both
%   expressed in the base frame.
%
%   J = robot_jacobian (THETA, MODEL, LINK_INDEX, POINT) evaluates the
%   Jacobian of an arbitrary POINT (3x1, base frame) that is rigidly attached
%   to link LINK_INDEX.  Joints after LINK_INDEX cannot move that point, so
%   their columns are zero.  The obstacle avoidance controller uses this to
%   push the critical link away from an obstacle.
%
%   [J, POS, AXES] = robot_jacobian (...) also returns the forward kinematics
%   from ROBOT_KINEMATICS so callers do not evaluate it twice.
%
%   The Jacobian is the exact derivative of the forward kinematics:
%
%       J(1:3,i) = AXES(:,i) x (POINT - POS(:,i))
%       J(4:6,i) = AXES(:,i)
%
%   where POS(:,i) is the base of link i, i.e. the point on joint i's axis.
%
%   See also ROBOT_KINEMATICS, PA10_MODEL, DAMPED_LEAST_SQUARES.

[pos, axes_world] = robot_kinematics(theta, model);

if nargin < 3 || isempty(link_index)
    link_index = model.n;
end
if nargin < 4 || isempty(point)
    point = pos(:, end);
end

J = point_jacobian(pos, axes_world, link_index, point);

end

%!test
%! % the analytic Jacobian must match a central finite difference of the FK
%! model = pa10_model();
%! theta = [0.35, -0.45, 0.40, -0.30, 0.55, 0.25, -0.20];
%! J = robot_jacobian(theta, model);
%! h = 1e-6;
%! Jn = zeros(6, model.n);
%! for i = 1:model.n
%!     tp = theta; tp(i) = tp(i) + h;
%!     tm = theta; tm(i) = tm(i) - h;
%!     pp = robot_kinematics(tp, model);
%!     pm = robot_kinematics(tm, model);
%!     Jn(1:3, i) = (pp(:, end) - pm(:, end)) / (2*h);
%! end
%! assert(J(1:3, :), Jn(1:3, :), 1e-6);

%!test
%! % the seventh joint is a pure roll at the tool point
%! model = pa10_model();
%! J = robot_jacobian([0.2, -0.3, 0.4, -0.5, 0.3, 0.2, 0.1], model);
%! assert(norm(J(1:3, 7)) < 1e-12);
%! assert(norm(J(4:6, 7)) > 0.9);

%!test
%! % joints after the attachment link cannot move an attached point
%! model = pa10_model();
%! theta = [0.2, -0.3, 0.4, -0.5, 0.3, 0.2, 0.1];
%! pos = robot_kinematics(theta, model);
%! point = 0.5 * (pos(:, 3) + pos(:, 4));
%! J = robot_jacobian(theta, model, 3, point);
%! assert(norm(J(:, 4:7)) == 0);
