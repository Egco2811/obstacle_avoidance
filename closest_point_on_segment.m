function [cp, t] = closest_point_on_segment(a, b, p)
%CLOSEST_POINT_ON_SEGMENT  Closest point of a segment to a point.
%
%   [CP, T] = closest_point_on_segment (A, B, P) returns the point CP on the
%   segment A-B that is closest to P, and the parameter T in [0,1] with
%   CP = A + T*(B - A).  A degenerate segment (A == B) is handled and returns
%   T = 0.
%
%   See also LINE_TO_POINT_DISTANCE, SEGMENT_SEGMENT_DISTANCE.

a = a(:);
b = b(:);
p = p(:);

ab = b - a;
len_sq = ab.' * ab;

if len_sq < eps
    t = 0;
else
    t = (p - a).' * ab / len_sq;
    t = min(1, max(0, t));
end

cp = a + t * ab;

end

%!test
%! [cp, t] = closest_point_on_segment([0;0;0], [1;0;0], [0.25; 1; 0]);
%! assert(cp, [0.25; 0; 0], 1e-12);
%! assert(t, 0.25, 1e-12);

%!test
%! % projections outside the segment clamp to the end points
%! [cp, ~] = closest_point_on_segment([0;0;0], [1;0;0], [-2; 0; 0]);
%! assert(cp, [0; 0; 0], 1e-12);
%! [cp, ~] = closest_point_on_segment([0;0;0], [1;0;0], [5; 0; 0]);
%! assert(cp, [1; 0; 0], 1e-12);

%!test
%! % degenerate segment
%! [cp, t] = closest_point_on_segment([1;2;3], [1;2;3], [4;6;3]);
%! assert(cp, [1; 2; 3], 1e-12);
%! assert(t, 0);
