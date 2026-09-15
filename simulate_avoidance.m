function log = simulate_avoidance(scenario, options)
%SIMULATE_AVOIDANCE  Run one obstacle avoidance scenario and log the result.
%
%   LOG = simulate_avoidance (SCENARIO) integrates the arm forward in time
%   using AVOIDANCE_CONTROLLER at every step.  SCENARIO comes from
%   EXAMPLE_SCENARIOS.  The simulation works headless, which is what the test
%   suite uses; pass an animation option to watch it.
%
%   LOG = simulate_avoidance (SCENARIO, OPTIONS) accepts
%
%     animate          draw the animation (default true)
%     figure_visible   show the window (default true)
%     scene            extra options forwarded to ROBOTSCENE
%     record           directory for PNG frames, empty to disable
%     record_every     save every N-th frame (default 4)
%     frame_pause      seconds to wait after each frame (default 0)
%     end_pause        seconds to keep the final frame on screen
%     strategy         controller strategy, see AVOIDANCE_CONTROLLER
%     damping          controller damping
%     avoid_gain       escape speed at full violation depth [m/s]
%     max_escape_ratio clamp on the escape speed, as a multiple of avoid_gain
%     tangential_weight how much the escape slides around the obstacle
%     task_weight      tracking weight relative to the escape task
%     max_joint_speed  joint velocity clamp [rad/s]
%     goal_gain        proportional gain of the goal task [1/s]
%     max_task_speed   clamp on the tool speed of the goal task [m/s]
%     goal_tolerance   distance at which the goal counts as reached [m]
%     default_safety   safety margin for obstacles that leave it empty
%     force            stop instead of warning when the figure cannot open
%     verbose          print a per-step line when the controller state changes
%
%   The returned LOG has the fields
%     time         1 x steps
%     theta        N x steps joint angles
%     tip          3 x steps tool centre point
%     clearance    1 x steps smallest surface clearance [m]
%     violation    1 x steps largest violation depth [m]
%     active       1 x steps logical, true while avoiding
%     link         1 x steps violating link index, NaN when clear
%     mode         cell array of controller states
%     tip_speed    1 x steps tool speed [m/s]
%     reached      true once the goal tolerance was met
%     summary      struct with the headline numbers
%     scene_handle the RobotScene, empty when not animating
%
%   See also EXAMPLE_SCENARIOS, AVOIDANCE_CONTROLLER, ROBOTSCENE, RUN_EXAMPLE.

if nargin < 1 || isempty(scenario)
    scenario = example_scenarios('single_sphere');
end
if nargin < 2 || isempty(options)
    options = struct();
end
options = simulate_defaults(options, scenario);

model = pa10_model();
theta = scenario.initial_theta(:).';
dt = scenario.dt;
steps = scenario.steps;

log = struct();
log.time = zeros(1, steps);
log.theta = zeros(model.n, steps);
log.tip = zeros(3, steps);
log.clearance = zeros(1, steps);
log.violation = zeros(1, steps);
log.active = false(1, steps);
log.link = nan(1, steps);
log.mode = cell(1, steps);
log.tip_speed = zeros(1, steps);
log.reached = false;
log.scene_handle = [];

scene = [];
if options.animate
    scene_options = options.scene;
    scene_options.visible = options.figure_visible;
    if isempty(options.record)
        try
            scene = RobotScene(model, scenario, scene_options);
        catch error_info
            if options.force
                rethrow(error_info);
            end
            warning('simulate_avoidance:noGraphics', ...
                'Could not open a figure (%s). Continuing without animation.', ...
                error_info.message);
            scene = [];
        end
    else
        scene = RobotScene(model, scenario, scene_options);
    end
end

if ~isempty(scene)
    log.scene_handle = scene;
end

if ~isempty(options.record) && ~exist(options.record, 'dir')
    mkdir(options.record);
end

previous_mode = '';
goal_reached = false;

for step = 1:steps
    elapsed = (step - 1) * dt;

    tool_point = robot_kinematics(theta, model);
    task_velocity = task_for(scenario, tool_point(:, end), goal_reached, options);

    controller_options = struct( ...
        'task_velocity', task_velocity, ...
        'strategy', options.strategy, ...
        'damping', options.damping, ...
        'avoid_gain', options.avoid_gain, ...
        'max_escape_ratio', options.max_escape_ratio, ...
        'tangential_weight', options.tangential_weight, ...
        'task_weight', options.task_weight, ...
        'max_joint_speed', options.max_joint_speed, ...
        'default_safety', options.default_safety);

    [dtheta, info] = avoidance_controller( ...
        theta, model, scenario.obstacles, controller_options, elapsed);

    if ~isempty(scenario.goal) && ...
            norm(scenario.goal(:) - info.pos(:, end)) < options.goal_tolerance
        goal_reached = true;
    end

    log.time(step) = elapsed;
    log.theta(:, step) = theta(:);
    log.tip(:, step) = info.pos(:, end);
    log.clearance(step) = min([info.clearance.clearance]);
    log.violation(step) = max(info.violation);
    log.active(step) = info.active;
    log.link(step) = info.link;
    log.mode{step} = info.mode;
    log.tip_speed(step) = norm(info.tip_velocity(1:3));

    if ~isempty(scene)
        scene.draw(theta, info, step, steps, elapsed);
        if options.frame_pause > 0
            pause(options.frame_pause);
        end
    end

    if ~isempty(options.record) && mod(step - 1, options.record_every) == 0
        scene.save_frame(fullfile(options.record, sprintf('frame_%04d.png', step)));
    end

    if options.verbose && ~strcmp(info.mode, previous_mode)
        fprintf('step %3d  t = %5.1f s  %s  clearance %+.3f m\n', ...
            step, elapsed, info.mode, log.clearance(step));
        previous_mode = info.mode;
    end

    theta = theta + dt * dtheta.';
end

log.reached = goal_reached;
log.summary = summarise(log, model, scenario, options);

if options.verbose
    fprintf('\n%s\n', log.summary.text);
end

if ~isempty(scene) && options.end_pause > 0
    pause(options.end_pause);
end

end

function task_velocity = task_for(scenario, tool_point, goal_reached, options)
%TASK_FOR  Desired tool twist for this step, in the base frame.

if isempty(scenario.goal) || goal_reached
    task_velocity = scenario.task_velocity;
    return;
end

error_vector = scenario.goal(:) - tool_point(:);
velocity = options.goal_gain * error_vector;
speed = norm(velocity);
if speed > options.max_task_speed
    velocity = velocity * (options.max_task_speed / speed);
end

task_velocity = [velocity; 0; 0; 0];

end

function summary = summarise(log, model, scenario, options)
%SUMMARISE  Headline numbers for the run, printed and returned.

[minimum_clearance, worst_step] = min(log.clearance);
final_error = NaN;
if ~isempty(scenario.goal)
    final_error = norm(scenario.goal(:) - log.tip(:, end));
end

violated = log.clearance < 0;
summary = struct();
summary.name = scenario.name;
summary.steps = numel(log.time);
summary.duration = log.time(end);
summary.min_clearance = minimum_clearance;
summary.min_clearance_time = log.time(worst_step);
summary.max_violation = max(log.violation);
summary.steps_avoiding = sum(log.active);
summary.entered_obstacle = any(violated);
summary.final_goal_error = final_error;
summary.reached = log.reached;

summary.text = sprintf([ ...
    'scenario "%s": %d steps, %.1f s\n', ...
    '  smallest clearance %+.3f m at t = %.1f s\n', ...
    '  deepest safety violation %+.3f m, %d steps spent avoiding\n', ...
    '  final goal error %.3f m, reached = %d'], ...
    summary.name, summary.steps, summary.duration, ...
    summary.min_clearance, summary.min_clearance_time, ...
    summary.max_violation, summary.steps_avoiding, ...
    summary.final_goal_error, summary.reached);

if summary.entered_obstacle
    summary.text = sprintf('%s\n  WARNING: a link entered an obstacle body', summary.text);
end

end

function options = simulate_defaults(options, scenario)
%SIMULATE_DEFAULTS  Fill in missing simulation options.

defaults = struct( ...
    'animate', true, ...
    'figure_visible', true, ...
    'scene', struct(), ...
    'record', '', ...
    'record_every', 4, ...
    'frame_pause', 0, ...
    'end_pause', 0, ...
    'strategy', 'full_chain', ...
    'damping', 0.02, ...
    'avoid_gain', 0.06, ...
    'max_escape_ratio', 4, ...
    'tangential_weight', 1.0, ...
    'task_weight', 0.35, ...
    'max_joint_speed', 0.9, ...
    'goal_gain', 1.8, ...
    'max_task_speed', 0.30, ...
    'goal_tolerance', 0.02, ...
    'default_safety', 0.05, ...
    'force', false, ...
    'verbose', true);

names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(options, name)
        options.(name) = defaults.(name);
    end
end

if ~isfield(options.scene, 'subtitle')
    options.scene.subtitle = scenario.description;
end
if ~isfield(options.scene, 'title')
    options.scene.title = scenario.name;
end
if ~isfield(options.scene, 'window_title')
    options.scene.window_title = sprintf('Obstacle avoidance: %s', scenario.name);
end

end

%!test
%! % end to end: a headless run must never let a link enter an obstacle body
%! scenario = example_scenarios('single_sphere');
%! scenario.steps = 120;
%! log = simulate_avoidance(scenario, struct('animate', false, 'verbose', false));
%! assert(numel(log.time), 120);
%! assert(size(log.theta, 1), 7);
%! assert(all(log.clearance > 0));
%! assert(log.summary.entered_obstacle, false);

%!test
%! % every example is runnable and collision free, on a shortened horizon
%! names = {'single_sphere', 'sphere_gate', 'crossbar', 'legacy'};
%! for k = 1:numel(names)
%!     scenario = example_scenarios(names{k});
%!     scenario.steps = 140;
%!     log = simulate_avoidance(scenario, struct('animate', false, 'verbose', false));
%!     assert(all(log.clearance > 0));
%!     assert(log.summary.min_clearance > 0);
%! end

%!test
%! % the safety shell of an obstacle is respected to the requested margin
%! scenario = example_scenarios('crossbar');
%! scenario.steps = 200;
%! log = simulate_avoidance(scenario, struct('animate', false, 'verbose', false));
%! % the arm may enter the margin, that is what the margin is for, but it must
%! % not go deeper than the margin itself
%! assert(max(log.violation) < max([scenario.obstacles.safety]));
