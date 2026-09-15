%RUN_ALL_EXAMPLES  Run every obstacle avoidance example and summarise them.
%
%   This is a script, not a function, so the options below can be edited or
%   pre-set before running it:
%
%       animate = true;      % show each scenario in a window
%       record_dir = '';     % e.g. 'output' to write PNG frames
%       run_all_examples
%
%   With ANIMATE left false the whole sweep runs headless, which is a quick
%   way to check that no scenario lets a link enter an obstacle body.
%
%   See also RUN_EXAMPLE, EXAMPLE_SCENARIOS, SIMULATE_AVOIDANCE.

if exist('animate', 'var') ~= 1 || isempty(animate)
    animate = false;
end
if exist('record_dir', 'var') ~= 1 || isempty(record_dir)
    record_dir = '';
end

scenarios = example_scenarios();
results = repmat(struct( ...
    'name', '', 'min_clearance', 0, 'max_violation', 0, ...
    'steps_avoiding', 0, 'reached', false, 'goal_error', NaN, ...
    'entered_obstacle', false), 1, numel(scenarios));

fprintf('%d examples\n\n', numel(scenarios));

for k = 1:numel(scenarios)
    scenario = scenarios(k);
    options = struct('animate', animate, 'verbose', false, 'end_pause', 0.4);
    if ~isempty(record_dir)
        options.record = fullfile(record_dir, scenario.name);
    end

    log = simulate_avoidance(scenario, options);

    results(k).name = scenario.name;
    results(k).min_clearance = log.summary.min_clearance;
    results(k).max_violation = log.summary.max_violation;
    results(k).steps_avoiding = log.summary.steps_avoiding;
    results(k).reached = log.summary.reached;
    results(k).goal_error = log.summary.final_goal_error;
    results(k).entered_obstacle = log.summary.entered_obstacle;

    fprintf('%-15s  min clearance %+7.3f m   deepest violation %+6.3f m   ', ...
        results(k).name, results(k).min_clearance, results(k).max_violation);
    if results(k).entered_obstacle
        fprintf('COLLISION\n');
    else
        fprintf('ok\n');
    end
end

fprintf('\n%-15s %10s %10s %8s %8s %10s\n', ...
    'scenario', 'clearance', 'violation', 'avoiding', 'reached', 'goal err');
fprintf('%s\n', repmat('-', 1, 68));
for k = 1:numel(results)
    fprintf('%-15s %+10.3f %+10.3f %8d %8d %10.3f\n', ...
        results(k).name, results(k).min_clearance, results(k).max_violation, ...
        results(k).steps_avoiding, results(k).reached, results(k).goal_error);
end
