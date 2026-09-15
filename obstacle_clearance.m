function [info, resolved] = obstacle_clearance(pos, obstacles, t, default_safety)
%OBSTACLE_CLEARANCE  Signed clearance between every link and every obstacle.
%
%   INFO = obstacle_clearance (POS, OBSTACLES) evaluates, for each obstacle,
%   the link of the arm that comes closest to it.  POS is the forward
%   kinematics from ROBOT_KINEMATICS, i.e. 3x(N+1).
%
%   INFO is a struct array with one entry per obstacle:
%     clearance       distance between the surfaces, negative when the arm is
%                     already inside the obstacle [m]
%     margin          the obstacle's safety margin [m]
%     link            index of the closest link
%     s               parameter in [0,1] of the closest point on that link
%     robot_point     closest point on the link, base frame
%     obstacle_point  closest point on the obstacle surface region, base frame
%     normal          unit vector from OBSTACLE_POINT to ROBOT_POINT
%     obstacle_index  index into OBSTACLES
%
%   A link violates its safety margin when CLEARANCE < MARGIN.  The violation
%   depth, MARGIN - CLEARANCE, is the quantity the controller reacts to.
%
%   [INFO, RESOLVED] = obstacle_clearance (POS, OBSTACLES, T) evaluates moving
%   obstacles at time T.  An obstacle moves when its MOTION field holds a
%   function handle @(OBSTACLE, T) that returns the obstacle at time T.
%   RESOLVED is the obstacle array at time T, which the renderer uses so the
%   drawing and the physics never disagree.
%
%   ... = obstacle_clearance (POS, OBSTACLES, T, DEFAULT_SAFETY) uses
%   DEFAULT_SAFETY for obstacles whose SAFETY field is empty.
%
%   See also SPHERE_OBSTACLE, CAPSULE_OBSTACLE, CLOSEST_POINT_ON_SEGMENT,
%   SEGMENT_SEGMENT_DISTANCE.

if nargin < 3 || isempty(t)
    t = 0;
end
if nargin < 4 || isempty(default_safety)
    default_safety = 0.05;
end

n_links = size(pos, 2) - 1;
n_obstacles = numel(obstacles);

info = repmat(empty_info(), 1, n_obstacles);
resolved = obstacles;

for j = 1:n_obstacles
    obstacle = obstacles(j);

    if isfield(obstacle, 'motion') && ~isempty(obstacle.motion)
        obstacle = obstacle.motion(obstacle, t);
        resolved(j).center = obstacle.center;
        resolved(j).p1 = obstacle.p1;
        resolved(j).p2 = obstacle.p2;
    end

    if isempty(obstacle.safety)
        margin = default_safety;
    else
        margin = obstacle.safety;
    end

    best = empty_info();
    best.clearance = inf;
    best.margin = margin;
    best.obstacle_index = j;

    for k = 1:n_links
        start_point = pos(:, k);
        end_point = pos(:, k + 1);

        switch lower(obstacle.type)
            case 'sphere'
                [robot_point, s] = closest_point_on_segment( ...
                    start_point, end_point, obstacle.center);
                obstacle_point = obstacle.center(:);
            case 'capsule'
                [~, s, ~, robot_point, obstacle_point] = ...
                    segment_segment_distance(start_point, end_point, ...
                                             obstacle.p1, obstacle.p2);
            otherwise
                error('obstacle_clearance:unknownType', ...
                    'Unknown obstacle type "%s".', obstacle.type);
        end

        offset = robot_point - obstacle_point;
        distance = norm(offset);
        clearance = distance - obstacle.radius;

        if clearance < best.clearance
            best.clearance = clearance;
            best.link = k;
            best.s = s;
            best.robot_point = robot_point;
            best.obstacle_point = obstacle_point;
            if distance < eps
                best.normal = [0; 0; 1];
            else
                best.normal = offset / distance;
            end
        end
    end

    info(j) = best;
end

end

function info = empty_info()
%EMPTY_INFO  Field order shared by every INFO entry.
info = struct( ...
    'clearance', inf, ...
    'margin', [], ...
    'link', 1, ...
    's', 0, ...
    'robot_point', [0; 0; 0], ...
    'obstacle_point', [0; 0; 0], ...
    'normal', [0; 0; 1], ...
    'obstacle_index', 1);
end

%!test
%! % a point obstacle far away from a straight arm
%! pos = [zeros(3, 2), [1; 0; 0]];
%! o = sphere_obstacle([0.5; 0.5; 0], 0.1, 0.05);
%! info = obstacle_clearance(pos, o);
%! assert(info.clearance, 0.4, 1e-12);
%! assert(info.link, 2);
%! assert(info.robot_point, [0.5; 0; 0], 1e-12);

%!test
%! % a capsule parallel to a link keeps a constant clearance
%! pos = [zeros(3, 2), [1; 0; 0]];
%! o = capsule_obstacle([0; 0.5; 0], [1; 0.5; 0], 0.1);
%! info = obstacle_clearance(pos, o);
%! assert(info.clearance, 0.4, 1e-9);

%!test
%! % violation depth is positive once the arm is inside the safety margin
%! pos = [zeros(3, 2), [1; 0; 0]];
%! o = sphere_obstacle([0.5; 0.15; 0], 0.1, 0.1);
%! info = obstacle_clearance(pos, o);
%! assert(info.clearance, 0.05, 1e-12);
%! assert(info.margin - info.clearance, 0.05, 1e-12);

%!test
%! % moving obstacles are resolved at the requested time
%! pos = [zeros(3, 2), [1; 0; 0]];
%! o = sphere_obstacle([0.5; 0.5; 0], 0.1);
%! o.motion = @(ob, t) setfield(ob, 'center', [0.5; 0.5 - 0.4 * t; 0]);
%! [info, resolved] = obstacle_clearance(pos, o, 1.0);
%! assert(resolved.center, [0.5; 0.1; 0], 1e-12);
%! assert(info.clearance, 0.0, 1e-12);

%!test
%! % mixed sphere/capsule arrays keep a consistent field layout
%! pos = [zeros(3, 2), [1; 0; 0]];
%! obs = [sphere_obstacle([0; 0; 1], 0.1), capsule_obstacle([0;0;0], [0;1;0], 0.1)];
%! info = obstacle_clearance(pos, obs);
%! assert(numel(info), 2);
%! assert(info(2).clearance, -0.1, 1e-12);
