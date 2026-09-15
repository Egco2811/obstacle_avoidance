function summary = run_all_tests()
%RUN_ALL_TESTS  Run every unit test in the project and print a summary.
%
%   SUMMARY = run_all_tests () runs the %!test blocks of every module and
%   returns a struct with the per module pass and fail counts.  It exits with
%   an error if anything failed, so it can be used as a check before a commit.
%
%   The tests cover the pieces the simulation is built from:
%
%     geometry    ClosestPointOnSegment, SegmentSegmentDistance,
%                 LineToPointDistance, CapsuleSurface, SphereSurface
%     kinematics  RobotKinematics, RobotJacobian, PointJacobian, and the
%                 fidelity check against the original MATLAB rotation scheme
%     control     DampedLeastSquares, ObstacleClearance, AvoidanceController
%     end to end  SimulateAvoidance over the example scenarios, checking that
%                 no link ever enters an obstacle body
%
%   See also TEST_MODEL_FIDELITY, ORIGINAL_MATLAB_SCHEME.

here = fileparts(mfilename('fullpath'));
addpath(here);

modules = { ...
    'SkewSymmetric', 'RotateBy', ...
    'closest_point_on_segment', 'segment_segment_distance', 'LineToPointDistance', ...
    'capsule_surface', 'sphere_surface', ...
    'robot_kinematics', 'robot_jacobian', 'point_jacobian', ...
    'test_model_fidelity', ...
    'damped_least_squares', 'obstacle_clearance', 'avoidance_controller', ...
    'example_scenarios', 'simulate_avoidance'};

summary = struct('name', {}, 'passed', {}, 'failed', {});
total_passed = 0;
total_failed = 0;

fprintf('running %d test modules\n\n', numel(modules));

for k = 1:numel(modules)
    name = modules{k};
    try
        % Octave returns [N, NMAX, ...]: tests passed, and tests run.
        [passed, run_count] = test(name, 'quiet');
        failed = run_count - passed;
        status = 'ok';
    catch error_info
        passed = 0;
        failed = 1;
        run_count = 1;
        status = error_info.message;
    end
    summary(k).name = name;
    summary(k).passed = passed;
    summary(k).failed = failed;
    total_passed = total_passed + passed;
    total_failed = total_failed + failed;
    fprintf('%-26s %3d passed  %3d failed   %s\n', name, passed, failed, status);
end

fprintf('\n%d passed, %d failed\n', total_passed, total_failed);

if total_failed > 0
    error('run_all_tests:failed', '%d test(s) failed.', total_failed);
end

end
