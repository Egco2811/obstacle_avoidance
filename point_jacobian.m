function J = point_jacobian(pos, axes_world, link_index, point)
%POINT_JACOBIAN  Jacobian of a point attached to one link.
%
%   J = point_jacobian (POS, AXES, LINK_INDEX, POINT) builds the 6xN
%   geometric Jacobian of POINT, expressed in the base frame, when POINT is
%   rigidly attached to link LINK_INDEX.  POS and AXES come from
%   ROBOT_KINEMATICS, so the forward kinematics is not evaluated twice when
%   several points of the same configuration are needed.
%
%   Joints after LINK_INDEX cannot move the point, so their columns are zero.
%   For the tool centre point use LINK_INDEX = N and POINT = POS(:, end); the
%   convenience wrapper ROBOT_JACOBIAN does exactly that.
%
%   See also ROBOT_JACOBIAN, ROBOT_KINEMATICS, AVOIDANCE_CONTROLLER.

n = size(axes_world, 2);
link_index = min(max(round(link_index), 1), n);
point = point(:);

J = zeros(6, n);
for i = 1:link_index
    J(1:3, i) = cross(axes_world(:, i), point - pos(:, i));
    J(4:6, i) = axes_world(:, i);
end

end

%!test
%! model = pa10_model();
%! theta = [0.31, -0.42, 0.37, -0.28, 0.52, 0.22, -0.17];
%! [pos, axes] = robot_kinematics(theta, model);
%! J = point_jacobian(pos, axes, 5, pos(:, 6));
%! assert(size(J), [6, 7]);
%! assert(norm(J(:, 6:7)), 0);
