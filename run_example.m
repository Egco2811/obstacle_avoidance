function log = run_example(name, options)
%RUN_EXAMPLE  Run one named obstacle avoidance example.
%
%   LOG = run_example (NAME) runs the scenario called NAME from
%   EXAMPLE_SCENARIOS and animates it.  LOG is the output of
%   SIMULATE_AVOIDANCE.
%
%   LOG = run_example (NAME, OPTIONS) forwards OPTIONS to SIMULATE_AVOIDANCE,
%   so for example
%
%       run_example('crossbar', struct('animate', false))
%
%   runs the crossbar scenario without opening a window, and
%
%       run_example('crossbar', struct('record', 'output/crossbar'))
%
%   also writes PNG frames of every fourth step into output/crossbar.
%
%   Run every example with RUN_ALL_EXAMPLES, and see EXAMPLE_SCENARIOS for the
%   list of names.

if nargin < 1 || isempty(name)
    name = 'single_sphere';
end
if nargin < 2
    options = struct();
end

log = simulate_avoidance(example_scenarios(name), options);

end
