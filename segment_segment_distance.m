function [dist, s, t, c1, c2] = segment_segment_distance(p1, q1, p2, q2)
%SEGMENT_SEGMENT_DISTANCE  Shortest distance between two 3-D segments.
%
%   [DIST, S, T, C1, C2] = segment_segment_distance (P1, Q1, P2, Q2) returns
%   the shortest distance DIST between the segments P1-Q1 and P2-Q2, the
%   parameters S and T in [0,1] of the closest points, and the closest points
%   themselves, C1 = P1 + S*(Q1 - P1) and C2 = P2 + T*(Q2 - P2).
%
%   This is the clamped closest-point algorithm from Ericson, "Real-Time
%   Collision Detection", section 5.1.9, so parallel and degenerate segments
%   are handled without division by zero.
%
%   See also CLOSEST_POINT_ON_SEGMENT, OBSTACLE_CLEARANCE.

p1 = p1(:); q1 = q1(:);
p2 = p2(:); q2 = q2(:);

d1 = q1 - p1;
d2 = q2 - p2;
r = p1 - p2;

a = d1.' * d1;
e = d2.' * d2;
f = d2.' * r;

tol = 1e-12;
if a <= tol && e <= tol
    s = 0;
    t = 0;
elseif a <= tol
    s = 0;
    t = min(1, max(0, f / e));
else
    c = d1.' * r;
    if e <= tol
        t = 0;
        s = min(1, max(0, -c / a));
    else
        b = d1.' * d2;
        denom = a * e - b * b;
        if denom > tol
            s = min(1, max(0, (b * f - c * e) / denom));
        else
            s = 0;   % parallel segments: pick an arbitrary start
        end
        t = (b * s + f) / e;
        if t < 0
            t = 0;
            s = min(1, max(0, -c / a));
        elseif t > 1
            t = 1;
            s = min(1, max(0, (b - c) / a));
        end
    end
end

c1 = p1 + s * d1;
c2 = p2 + t * d2;
dist = norm(c1 - c2);

end

%!test
%! [d, s, t, c1, c2] = segment_segment_distance([0;0;0], [1;0;0], [0;1;0], [1;1;0]);
%! assert(d, 1, 1e-12);
%! assert(c1(2), 0, 1e-12);
%! assert(c2(2), 1, 1e-12);

%!test
%! % crossing segments touch
%! [d, ~, ~, ~, ~] = segment_segment_distance([-1;0;0], [1;0;0], [0;-1;0], [0;1;0]);
%! assert(d, 0, 1e-12);

%!test
%! % parallel segments are not degenerate
%! [d, ~, ~, ~, ~] = segment_segment_distance([0;0;0], [1;0;0], [0;0;2], [1;0;2]);
%! assert(d, 2, 1e-12);

%!test
%! % against a brute force sampling in a generic configuration
%! p1 = [0; 0; 0]; q1 = [0.7; 0.2; -0.1];
%! p2 = [0.3; 0.5; 0.4]; q2 = [-0.2; 0.1; 0.6];
%! d = segment_segment_distance(p1, q1, p2, q2);
%! best = inf;
%! for i = 0:200
%!     for j = 0:200
%!         a = p1 + (i/200) * (q1 - p1);
%!         b = p2 + (j/200) * (q2 - p2);
%!         best = min(best, norm(a - b));
%!     end
%! end
%! assert(d <= best + 1e-3);
