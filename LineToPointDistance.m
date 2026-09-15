function distance = LineToPointDistance(line_end, line_begin, point)
%LINE_TO_POINT_DISTANCE  Distance from a point to a line segment.
%
%   D = LineToPointDistance (LINE_END, LINE_BEGIN, POINT) returns the distance
%   from POINT to the segment that runs from LINE_BEGIN to LINE_END.  The
%   argument order is kept from the original MATLAB proof of concept.
%
%   Callers that also need the closest point itself should use
%   CLOSEST_POINT_ON_SEGMENT instead, which returns both.
%
%   Created by Egco 20/05/2025
%
%   See also CLOSEST_POINT_ON_SEGMENT, OBSTACLE_CLEARANCE.

point = point(:);
cp = closest_point_on_segment(line_begin, line_end, point);
distance = norm(cp - point);

end

%!test
%! assert(LineToPointDistance([1;0;0], [0;0;0], [0.5; 0.25; 0]), 0.25, 1e-12);
%! assert(LineToPointDistance([1;0;0], [0;0;0], [2; 0; 0]), 1, 1e-12);
%! assert(LineToPointDistance([1;0;0], [0;0;0], [-3; 0; 0]), 3, 1e-12);
