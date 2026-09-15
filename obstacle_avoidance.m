%OBSTACLE_AVOIDANCE  7-DOF manipulator obstacle avoidance demo (GNU Octave).
%
%   This is the entry point of the project.  It runs one scenario from
%   EXAMPLE_SCENARIOS, animates the arm, and prints a summary of how close the
%   links came to the obstacles.
%
%   Running it
%   ----------
%       octave --gui obstacle_avoidance.m
%
%   Set SCENARIO_NAME before running to pick a different example, and set
%   RECORD_DIR to also write PNG frames of the animation:
%
%       scenario_name = 'crossbar';
%       obstacle_avoidance
%
%       scenario_name = 'moving_sphere';
%       record_dir = 'output/moving_sphere';
%       obstacle_avoidance
%
%   Type `example_scenarios` to list the available names.  RUN_ALL_EXAMPLES
%   sweeps through all of them and reports whether any link ever entered an
%   obstacle body.
%
%   How it works
%   ------------
%   The arm is the PA-10 based 7-DOF manipulator described in PA10_MODEL.
%   Every step the tool is asked to move towards the goal, and the joint
%   velocities that realise that motion come from AVOIDANCE_CONTROLLER, which
%   is a resolved rate law built on the geometric Jacobian of ROBOT_JACOBIAN.
%   Whenever a link is inside an obstacle's safety margin the controller adds
%   an escape task at the closest point of that link, so the arm is pushed
%   away from the obstacle while it keeps tracking the goal.
%
%   See also EXAMPLE_SCENARIOS, RUN_EXAMPLE, RUN_ALL_EXAMPLES,
%   SIMULATE_AVOIDANCE, AVOIDANCE_CONTROLLER, ROBOTSCENE.

if exist('scenario_name', 'var') ~= 1 || isempty(scenario_name)
    scenario_name = 'single_sphere';
end
if exist('record_dir', 'var') ~= 1 || isempty(record_dir)
    record_dir = '';
end

log = simulate_avoidance(example_scenarios(scenario_name), ...
    struct('record', record_dir));

fprintf('The animated figure lives in log.scene_handle; clear log to close it.\n');
