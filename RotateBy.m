function RotationMatrix = RotateBy(axis, radian)
%ROTATEBY  Rotation matrix about an arbitrary axis (Rodrigues' formula).
%
%   R = RotateBy (AXIS, ANGLE) returns the 3x3 rotation matrix that rotates a
%   vector by ANGLE radians about AXIS.  AXIS does not have to be a unit
%   vector; it is normalised here.
%
%   Created by Egco 21/05/2025
%
%   See also SKEWSYMMETRIC, ROBOT_KINEMATICS.

axis = axis(:);
if numel(axis) ~= 3
    error('RotateBy:badAxis', 'AXIS must have 3 elements.');
end
axis = axis / norm(axis);

khat = SkewSymmetric(axis);
RotationMatrix = eye(3) + sin(radian) * khat + (1 - cos(radian)) * (khat * khat);

end

%!test
%! % a quarter turn about z takes x to y
%! R = RotateBy([0; 0; 1], pi/2);
%! assert(R * [1; 0; 0], [0; 1; 0], 1e-12);
%! assert(R * [0; 1; 0], [-1; 0; 0], 1e-12);

%!test
%! % the axis itself is left alone and angles add up
%! axis = [1; 2; -1];
%! assert(RotateBy(axis, 1.0) * axis, axis, 1e-12);
%! combined = RotateBy(axis, 0.4) * RotateBy(axis, 0.6);
%! assert(combined, RotateBy(axis, 1.0), 1e-12);

%!test
%! % a rotation is orthogonal with unit determinant
%! R = RotateBy([0.3; -0.4; 0.9], 2.1);
%! assert(R * R.', eye(3), 1e-12);
%! assert(det(R), 1, 1e-12);

%!test
%! % and it preserves lengths and angles
%! R = RotateBy([1; 1; 0], 0.7);
%! a = [1; 2; 3];
%! b = [-2; 0.5; 1];
%! assert(norm(R*a), norm(a), 1e-12);
%! assert(dot(R*a, R*b), dot(a, b), 1e-12);
