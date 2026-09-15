function obstacle = sphere_obstacle(center, radius, safety, color)
%SPHERE_OBSTACLE  Build a spherical obstacle description.
%
%   OBSTACLE = sphere_obstacle (CENTER, RADIUS) describes a ball centred at
%   CENTER (3x1) with physical radius RADIUS.  The avoidance controller keeps
%   every link at least SAFETY away from the surface.
%
%   OBSTACLE = sphere_obstacle (CENTER, RADIUS, SAFETY, COLOR) also sets the
%   safety margin [m] and the RGB colour used for drawing.  Defaults are 0.05 m
%   and a warm red.
%
%   The returned struct is also produced by CAPSULE_OBSTACLE with the same
%   fields in the same order, so spheres and capsules can be concatenated into
%   one obstacle array:
%
%       obstacles = [sphere_obstacle([0.1;0.2;0.4], 0.03), ...
%                    capsule_obstacle([0;0;0.3], [0.5;0;0.3], 0.02)];
%
%   See also CAPSULE_OBSTACLE, OBSTACLE_CLEARANCE, EXAMPLE_SCENARIOS.

if nargin < 2 || isempty(radius), radius = 0.03; end
if nargin < 3 || isempty(safety), safety = 0.05; end
if nargin < 4 || isempty(color), color = [1.00, 0.35, 0.30]; end

center = center(:);
if numel(center) ~= 3
    error('sphere_obstacle:badCenter', 'CENTER must be a 3-element vector.');
end

obstacle = struct( ...
    'type', 'sphere', ...
    'center', center, ...
    'p1', [], ...
    'p2', [], ...
    'radius', radius, ...
    'safety', safety, ...
    'color', color, ...
    'motion', []);

end
