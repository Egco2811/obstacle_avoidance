function S = SkewSymmetric(vector)
%SKEWSYMMETRIC  Cross product matrix of a 3-D vector.
%
%   S = SkewSymmetric (VECTOR) returns the 3x3 matrix S with S * X ==
%   cross (VECTOR, X) for every X.  It is the standard building block for
%   rotation matrices and velocity Jacobians.
%
%   Created by Egco 21/05/2025
%
%   See also ROTATEBY, POINT_JACOBIAN.

vector = vector(:);
if numel(vector) ~= 3
    error('SkewSymmetric:badVector', 'VECTOR must have 3 elements.');
end

S = [0, -vector(3), vector(2); ...
     vector(3), 0, -vector(1); ...
     -vector(2), vector(1), 0];

end

%!test
%! % S * x must equal cross (v, x)
%! v = [0.3; -1.2; 0.7];
%! x = [2.0; 0.5; -1.5];
%! assert(SkewSymmetric(v) * x, cross(v, x), 1e-12);

%!test
%! % and the matrix must be skew symmetric
%! S = SkewSymmetric([1; 2; 3]);
%! assert(S, -S.', 1e-12);
%! assert(trace(S), 0, 1e-12);

%!test
%! S = SkewSymmetric([1; 2; 3]);
%! assert(S, [0, -3, 2; 3, 0, -1; -2, 1, 0], 1e-12);
