function scenarios = example_scenarios(name)
%EXAMPLE_SCENARIOS  The obstacle avoidance examples shipped with the project.
%
%   SCENARIOS = example_scenarios () returns a struct array with every example.
%   SCENARIO = example_scenarios (NAME) returns the one called NAME, or raises
%   an error listing the available names.
%
%   Run one with
%
%       run_example ('crossbar')
%
%   or start the main script on it with
%
%       scenario_name = 'crossbar';
%       obstacle_avoidance
%
%   Every scenario has the fields
%     name            short identifier
%     description     one line summary, shown in the window title
%     goal            3x1 tool target, empty for the velocity task
%     task_velocity   6x1 constant tool twist, used when GOAL is empty
%     obstacles       array built with SPHERE_OBSTACLE / CAPSULE_OBSTACLE
%     steps, dt       number of control steps and the step duration [s]
%     initial_theta   1xN starting joint angles [rad]
%     workspace       1x6 axis limits for the 3-D view
%     camera          1x2 [azimuth, elevation] for the 3-D view
%
%   See also RUN_EXAMPLE, SIMULATE_AVOIDANCE, SPHERE_OBSTACLE,
%   CAPSULE_OBSTACLE.

workspace = [-0.60, 0.60, -0.60, 0.60, -0.10, 1.10];
camera = [-40, 22];
initial_theta = [0.35, -0.45, 0.40, -0.30, 0.55, 0.25, 0.0];
steps = 420;
dt = 0.05;

scenarios = [];

% ---------------------------------------------------------------- one sphere
scenario = base_scenario('single_sphere', ...
    'One sphere directly in the path of the tool', ...
    [-0.40; 0.35; 0.45], ...
    sphere_obstacle([0.00; 0.10; 0.62], 0.045, 0.06, [0.88, 0.22, 0.20]));
scenarios = [scenarios, finish_scenario(scenario, workspace, camera, initial_theta, steps, dt)];

% ------------------------------------------------------------- a sphere gate
% Two spheres the tool has to thread between, which needs the whole arm to be
% steered around them rather than just the tool point.
scenario = base_scenario('sphere_gate', ...
    'Thread the tool between two spheres without clipping either', ...
    [-0.35; 0.45; 0.40], ...
    [sphere_obstacle([-0.02; -0.06; 0.60], 0.050, 0.055, [0.88, 0.22, 0.20]), ...
     sphere_obstacle([-0.02; 0.30; 0.60], 0.050, 0.055, [0.92, 0.52, 0.08])]);
scenarios = [scenarios, finish_scenario(scenario, workspace, camera, initial_theta, 320, dt)];

% ----------------------------------------------------------- moving obstacle
% The sphere sweeps across the workspace while the arm tries to reach the
% goal, so the arm has to keep re-planning its escape.
scenario = base_scenario('moving_sphere', ...
    'A sphere drifts back and forth across the path', ...
    [-0.40; 0.35; 0.45], ...
    sphere_obstacle([0.00; 0.10; 0.62], 0.045, 0.06, [0.88, 0.22, 0.20]));
scenario.obstacles(1).motion = @(obstacle, t) setfield(obstacle, 'center', ...
    [-0.05 + 0.22 * sin(0.55 * t); 0.12 - 0.20 * cos(0.40 * t); 0.58]);
scenarios = [scenarios, finish_scenario(scenario, workspace, camera, initial_theta, 460, dt)];

% ------------------------------------------------------------------- a beam
% A rod crossing the workspace at tool height: the arm has to pass above or
% below it, and the capsule primitive is what makes that visible.
scenario = base_scenario('crossbar', ...
    'Duck under a rod that crosses the workspace', ...
    [-0.40; 0.35; 0.35], ...
    capsule_obstacle([-0.10; -0.38; 0.615], [-0.10; 0.42; 0.615], ...
                     0.022, 0.05, [0.92, 0.52, 0.08]));
scenarios = [scenarios, finish_scenario(scenario, workspace, camera, initial_theta, 380, dt)];

% ----------------------------------------------------------- obstacle field
% Several small spheres, so the arm has to negotiate more than one constraint
% and the controller has to hand over from one violator to the next.
scenario = base_scenario('sphere_field', ...
    'Find a way through a field of small spheres', ...
    [-0.45; 0.45; 0.35], ...
    [sphere_obstacle([0.05; 0.00; 0.68], 0.035, 0.045, [0.88, 0.22, 0.20]), ...
     sphere_obstacle([-0.05; 0.25; 0.55], 0.035, 0.045, [0.92, 0.52, 0.08]), ...
     sphere_obstacle([-0.30; 0.05; 0.60], 0.035, 0.045, [0.62, 0.52, 0.05]), ...
     sphere_obstacle([0.10; 0.30; 0.35], 0.035, 0.045, [0.55, 0.25, 0.80])]);
scenarios = [scenarios, finish_scenario(scenario, workspace, camera, initial_theta, 460, dt)];

% ------------------------------------------------------------------- legacy
% The original MATLAB proof of concept: no goal, just a constant tool velocity
% in -x, and one sphere parked in the way of the second and third links.
scenario = base_scenario('legacy', ...
    'Original proof of concept: constant -x tool velocity', ...
    [], ...
    sphere_obstacle([0.22; 0.00; 0.71], 0.030, 0.070, [0.88, 0.22, 0.20]));
scenario.task_velocity = [-0.05; 0; 0; 0; 0; 0];
scenario = finish_scenario(scenario, workspace, camera, zeros(1, 7), 200, dt);
scenarios = [scenarios, scenario];

if nargin < 1 || isempty(name)
    return;
end

names = {scenarios.name};
index = find(strcmp(names, name), 1);
if isempty(index)
    error('example_scenarios:unknown', ...
        'Unknown scenario "%s". Available: %s.', name, strjoin(names, ', '));
end
scenarios = scenarios(index);

end

function scenario = base_scenario(name, description, goal, obstacles)
scenario = struct( ...
    'name', name, ...
    'description', description, ...
    'goal', goal, ...
    'task_velocity', zeros(6, 1), ...
    'obstacles', obstacles, ...
    'steps', 260, ...
    'dt', 0.05, ...
    'initial_theta', zeros(1, 7), ...
    'workspace', [-0.6, 0.6, -0.6, 0.6, -0.1, 1.1], ...
    'camera', [-40, 22]);
end

function scenario = finish_scenario(scenario, workspace, camera, initial_theta, steps, dt)
scenario.workspace = workspace;
scenario.camera = camera;
scenario.initial_theta = initial_theta;
scenario.steps = steps;
scenario.dt = dt;
end

%!test
%! scenarios = example_scenarios();
%! assert(numel(scenarios) >= 5);
%! names = {scenarios.name};
%! for k = 1:numel(names)
%!     single = example_scenarios(names{k});
%!     assert(strcmp(single.name, names{k}));
%!     assert(numel(single.obstacles) >= 1);
%!     assert(numel(single.initial_theta) == 7);
%! end

%!test
%! error_thrown = false;
%! try
%!     example_scenarios('no_such_scenario');
%! catch
%!     error_thrown = true;
%! end
%! assert(error_thrown);

%!test
%! % every scenario must hold obstacles of a single, concatenatable layout
%! scenarios = example_scenarios();
%! for k = 1:numel(scenarios)
%!     obstacles = scenarios(k).obstacles;
%!     for j = 1:numel(obstacles)
%!         assert(any(strcmp(obstacles(j).type, {'sphere', 'capsule'})));
%!         assert(isfield(obstacles(j), 'motion'));
%!     end
%! end
