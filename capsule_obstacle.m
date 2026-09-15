function obstacle = capsule_obstacle(p1, p2, radius, safety, color)
%CAPSULE_OBSTACLE  Build a rod shaped (capsule) obstacle description.
%
%   OBSTACLE = capsule_obstacle (P1, P2, RADIUS) describes a capsule: every
%   point whose distance to the segment P1-P2 is at most RADIUS.  It is the
%   right primitive for beams, rails, walls and any long thin geometry that a
%   sphere would model badly.
%
%   OBSTACLE = capsule_obstacle (P1, P2, RADIUS, SAFETY, COLOR) also sets the
%   safety margin [m] and the RGB colour used for drawing.
%
%   The returned struct has the same fields, in the same order, as the one
%   from SPHERE_OBSTACLE, so the two can be mixed in a single obstacle array.
%
%   See also SPHERE_OBSTACLE, OBSTACLE_CLEARANCE, EXAMPLE_SCENARIOS.

if nargin < 3 || isempty(radius), radius = 0.02; end
if nargin < 4 || isempty(safety), safety = 0.05; end
if nargin < 5 || isempty(color), color = [1.00, 0.65, 0.25]; end

p1 = p1(:);
p2 = p2(:);
if numel(p1) ~= 3 || numel(p2) ~= 3
    error('capsule_obstacle:badSegment', 'P1 and P2 must be 3-element vectors.');
end

obstacle = struct( ...
    'type', 'capsule', ...
    'center', [], ...
    'p1', p1, ...
    'p2', p2, ...
    'radius', radius, ...
    'safety', safety, ...
    'color', color, ...
    'motion', []);

end
