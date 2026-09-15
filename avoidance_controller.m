function [dtheta, info] = avoidance_controller(theta, model, obstacles, opts, t)
%AVOIDANCE_CONTROLLER  Resolved-rate control law with obstacle avoidance.
%
%   [DTHETA, INFO] = avoidance_controller (THETA, MODEL, OBSTACLES) returns the
%   joint velocities DTHETA (Nx1, rad/s) that should be commanded at joint
%   angles THETA, given a list of OBSTACLES built with SPHERE_OBSTACLE or
%   CAPSULE_OBSTACLE.
%
%   The controller works in two regimes.
%
%   Free space
%     No link is inside its safety margin, so the arm simply tracks the task
%     twist with a damped least squares inverse of the tool Jacobian:
%
%         dtheta = damped_least_squares (J_tool, task_velocity, damping)
%
%   Constrained space
%     At least one link is inside its safety margin.  For the worst violation
%     the controller builds an escape velocity at the closest point of the
%     offending link and solves for joint velocities that produce it while
%     still tracking the task as well as possible.
%     The two tasks are stacked into one damped least squares problem, so a
%     violation always wins over tracking but never has to be satisfied
%     exactly, which keeps the motion smooth.
%
%     The escape speed grows as the clearance shrinks,
%
%         speed = avoid_gain * (margin / clearance - 1)
%
%     which is zero exactly at the margin, and rises without bound as the arm
%     approaches the obstacle.  A constant speed cannot work here: it would
%     only ever cancel the component of the task that pushes the arm in, so
%     the arm would hover on the margin instead of leaving it.  The speed is
%     clamped at avoid_gain * max_escape_ratio so a deep violation cannot
%     produce a wild joint velocity.
%
%     The escape direction is the outward normal plus a tangential component
%     taken from the direction the tracking task already wants to move at that
%     point.  Without it the arm pushes straight back along the normal and
%     stalls; with it the arm slides around the obstacle and still gets to the
%     goal.
%
%   Two strategies are available in OPTS.strategy:
%
%     'freeze_base'
%       Joints before the violating link are held still, exactly like the
%       original proof of concept: the base of the arm keeps its pose and the
%       violating link does the escaping.  If those joints physically cannot
%       push the critical point away (the critical point sits on the joint
%       axis, or the joint is at a kinematic limit) the controller falls back
%       to the full chain on that step instead of stalling.
%
%     'full_chain' (default)
%       Every joint may contribute.  Usually gives the smoothest escape and
%       the smallest tracking error, at the price of no longer preserving the
%       pose of the arm below the obstacle.  Freezing the base joints leaves
%       the escape to a single joint, which is often not enough to push the
%       critical point clear, so the arm ends up creeping along the safety
%       boundary instead of going around the obstacle.
%
%   Options (struct, all optional)
%     task_velocity    6x1 desired tool twist [v; omega] in the base frame
%     strategy         'freeze_base' or 'full_chain'
%     damping          Levenberg-Marquardt damping for the least squares step
%     avoid_gain       escape speed at full violation depth [m/s]
%     max_escape_ratio clamp on the escape speed, as a multiple of avoid_gain
%     tangential_weight how much of the tracking direction is mixed into the
%                      escape direction
%     task_weight      weight of the tracking task relative to the escape task
%     max_joint_speed  clamp on the returned joint velocities [rad/s]
%     default_safety   safety margin used when an obstacle leaves it empty
%
%   INFO is a struct describing the step, and is what the renderer and the
%   caller need to log or draw:
%     pos, axes_world, frames  forward kinematics of the current pose
%     clearance, violation     per-obstacle OBSTACLE_CLEARANCE results and
%                              MARGIN - CLEARANCE, positive when violating
%     resolved_obstacles       obstacle list with moving obstacles evaluated
%     active                   true when some link is inside its margin
%     active_index             obstacle being escaped, NaN when inactive
%     nearest_index            obstacle the arm is closest to
%     link                     link that violates, NaN when inactive
%     escape_vector            commanded escape velocity at the critical point
%     escape_speed             its magnitude
%     tip_velocity             tool twist actually produced by DTHETA
%     joint_speed              norm of DTHETA
%     mode                     human readable controller state
%
%   See also DAMPED_LEAST_SQUARES, OBSTACLE_CLEARANCE, SIMULATE_AVOIDANCE.

if nargin < 4 || isempty(opts)
    opts = struct();
end
if nargin < 5 || isempty(t)
    t = 0;
end
opts = controller_defaults(opts);

[pos, axes_world, frames] = robot_kinematics(theta, model);
J_tool = point_jacobian(pos, axes_world, model.n, pos(:, end));

[near, resolved] = obstacle_clearance(pos, obstacles, t, opts.default_safety);
violation = [near.margin] - [near.clearance];
[worst_violation, worst_index] = max(violation);
[~, nearest_index] = min([near.clearance]);

info = struct();
info.pos = pos;
info.axes_world = axes_world;
info.frames = frames;
info.clearance = near;
info.violation = violation;
info.resolved_obstacles = resolved;
info.active = ~isempty(worst_violation) && worst_violation > 0;
info.active_index = NaN;
info.nearest_index = nearest_index;
info.link = NaN;
info.mode = 'track';
info.escape_speed = 0;
info.escape_vector = zeros(3, 1);

if ~info.active
    dtheta = damped_least_squares(J_tool, opts.task_velocity, opts.damping);
else
    worst = near(worst_index);
    info.link = worst.link;
    info.active_index = worst_index;

    % Only the linear part is constrained: there is no reason to also demand a
    % particular angular velocity at the critical point.
    J_critical = point_jacobian(pos, axes_world, worst.link, worst.robot_point);
    J_critical = J_critical(1:3, :);

    % Escape velocity at the closest point of the offending link.
    escape_velocity = escape_for(worst, J_critical, J_tool, opts);
    info.escape_vector = escape_velocity;
    info.escape_speed = norm(escape_velocity);

    [dtheta, info.mode] = solve_avoidance(J_critical, J_tool, ...
        escape_velocity, opts, worst.link);
end

dtheta = dtheta(:);
dtheta = min(opts.max_joint_speed, max(-opts.max_joint_speed, dtheta));
info.joint_speed = norm(dtheta);
info.tip_velocity = J_tool * dtheta;

end

function escape_velocity = escape_for(worst, J_critical, J_tool, opts)
%ESCAPE_FOR  Escape velocity at the closest point of a violating link.
%
%   The magnitude grows as the clearance shrinks, so the arm is pushed out of
%   the margin instead of being held on its edge, and the direction is tilted
%   towards wherever the tracking task wants to go so that the arm slides
%   around the obstacle rather than stalling head on.

clearance = worst.clearance;
margin = max(worst.margin, eps);

if clearance <= 0
    ratio = opts.max_escape_ratio;
else
    ratio = min(opts.max_escape_ratio, margin / clearance - 1);
end
ratio = max(ratio, 0);
speed = opts.avoid_gain * ratio;

normal = worst.normal;
direction = normal;

if opts.tangential_weight > 0 && speed > 0
    % Where would tracking alone try to move this point?
    dtheta_task = damped_least_squares(J_tool, opts.task_velocity, opts.damping);
    point_velocity = J_critical * dtheta_task;
    tangent = point_velocity - (point_velocity.' * normal) * normal;
    if norm(tangent) > 1e-9
        tangent = tangent / norm(tangent);
        direction = normal + opts.tangential_weight * tangent;
        direction = direction / norm(direction);
    end
end

escape_velocity = speed * direction;

end

function [dtheta, mode] = solve_avoidance(J_critical, J_tool, escape_velocity, opts, violating_link)
%SOLVE_AVOIDANCE  Escape task first, tracking task as well as possible.

n = size(J_tool, 2);
task_rows = opts.task_weight * J_tool;
task_rhs = opts.task_weight * opts.task_velocity;

if strcmpi(opts.strategy, 'freeze_base')
    active = violating_link:n;
    dq_active = damped_least_squares( ...
        [J_critical(:, active); task_rows(:, active)], ...
        [escape_velocity; task_rhs], opts.damping);

    dtheta = zeros(n, 1);
    dtheta(active) = dq_active;

    if norm(J_critical * dtheta) >= 0.5 * norm(escape_velocity)
        mode = sprintf('avoid: frozen below link %d', violating_link);
        return;
    end
    % The joints below the violation cannot move the critical point away.
    % Rather than stalling, let the whole chain contribute on this step.
    mode = sprintf('avoid: chain fallback for link %d', violating_link);
else
    mode = sprintf('avoid: full chain, link %d', violating_link);
end

dtheta = damped_least_squares([J_critical; task_rows], ...
    [escape_velocity; task_rhs], opts.damping);

end

function opts = controller_defaults(opts)
%CONTROLLER_DEFAULTS  Fill in missing controller options.

if ~isfield(opts, 'task_velocity') || isempty(opts.task_velocity)
    opts.task_velocity = zeros(6, 1);
end
if ~isfield(opts, 'strategy') || isempty(opts.strategy)
    opts.strategy = 'full_chain';
end
if ~isfield(opts, 'damping') || isempty(opts.damping)
    opts.damping = 0.02;
end
if ~isfield(opts, 'avoid_gain') || isempty(opts.avoid_gain)
    opts.avoid_gain = 0.06;
end
if ~isfield(opts, 'max_escape_ratio') || isempty(opts.max_escape_ratio)
    opts.max_escape_ratio = 4;
end
if ~isfield(opts, 'tangential_weight') || isempty(opts.tangential_weight)
    opts.tangential_weight = 1.0;
end
if ~isfield(opts, 'task_weight') || isempty(opts.task_weight)
    opts.task_weight = 0.35;
end
if ~isfield(opts, 'max_joint_speed') || isempty(opts.max_joint_speed)
    opts.max_joint_speed = 0.9;
end
if ~isfield(opts, 'default_safety') || isempty(opts.default_safety)
    opts.default_safety = 0.05;
end

opts.task_velocity = opts.task_velocity(:);
if numel(opts.task_velocity) ~= 6
    error('avoidance_controller:badTaskVelocity', ...
        'opts.task_velocity must have 6 elements.');
end

end

%!test
%! % with no obstacle in the way the tool follows the requested twist
%! model = pa10_model();
%! theta = [0.30, -0.40, 0.35, -0.30, 0.50, 0.20, 0.0];
%! obstacles = sphere_obstacle([0; -2; 0], 0.05);   % far away
%! opts = struct('task_velocity', [0.02; 0; 0; 0; 0; 0], 'damping', 1e-6);
%! [dtheta, info] = avoidance_controller(theta, model, obstacles, opts, 0);
%! assert(~info.active);
%! assert(info.mode, 'track');
%! assert(strcmp(info.mode, 'track'));

%!test
%! % an obstacle sitting on a link produces an escape motion away from it
%! model = pa10_model();
%! theta = zeros(1, model.n);
%! pos = robot_kinematics(theta, model);
%! % park the obstacle just above the middle of link 2
%! link_2_centre = 0.5 * (pos(:, 2) + pos(:, 3));
%! obstacles = sphere_obstacle(link_2_centre + [0; 0; 0.04], 0.01, 0.08);
%! opts = struct('task_velocity', zeros(6, 1), 'avoid_gain', 0.05);
%! [dtheta, info] = avoidance_controller(theta, model, obstacles, opts, 0);
%! assert(info.active);
%! assert(norm(dtheta) > 0);
%! % the critical point must move away from the obstacle, i.e. its velocity
%! % must be aligned with the outward normal
%! critical = info.clearance(1);
%! J = point_jacobian(info.pos, info.axes_world, critical.link, critical.robot_point);
%! critical_velocity = J(1:3, :) * dtheta;
%! assert(critical_velocity.' * critical.normal > 0);
%! assert(critical_velocity(3) < 0);

%!test
%! % the joint speed clamp is respected
%! model = pa10_model();
%! theta = zeros(1, model.n);
%! pos = robot_kinematics(theta, model);
%! obstacles = sphere_obstacle(pos(:, 3) + [0; 0; 0.01], 0.01, 0.2);
%! opts = struct('avoid_gain', 5, 'max_joint_speed', 0.5);
%! dtheta = avoidance_controller(theta, model, obstacles, opts, 0);
%! assert(norm(dtheta, inf) <= 0.5 + 1e-12);
