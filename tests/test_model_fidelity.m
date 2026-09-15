function test_model_fidelity()
%TEST_MODEL_FIDELITY  Port fidelity check for the kinematics.
%
%   The obstacle avoidance demo started life as a MATLAB script that rotated
%   its basis vectors in place, one round of small joint increments at a time.
%   The port replaces that with the closed form forward kinematics in
%   ROBOT_KINEMATICS.  This test proves the two agree, so the arm on screen
%   and the Jacobians used by the controller still describe the same machine.
%
%   Run it with:  test ('test_model_fidelity')
%
%   See also ROBOT_KINEMATICS, ORIGINAL_MATLAB_SCHEME.

end

%!test
%! % exact comparison: re-reading each joint axis after the previous joints
%! % have moved reproduces the closed form kinematics to machine precision
%! model = pa10_model();
%! dtheta = [0.4, -0.6, 0.9, -0.3, 0.5, -0.8, 0.2];
%! [pos_original, axes_original] = original_matlab_scheme(dtheta, 1, model, 'updated');
%! [pos_port, axes_port] = robot_kinematics(dtheta, model);
%! assert(pos_port, pos_original, 1e-12);
%! assert(axes_port, axes_original, 1e-12);

%!test
%! % the literal original scheme, with its stale axes, is a first order
%! % approximation: it converges to the ported kinematics as the step shrinks
%! model = pa10_model();
%! dtheta_step = 1e-4 * [1, -1, 0.7, -0.5, 0.9, -0.8, 0.3];
%! rounds = 4000;
%! pos_original = original_matlab_scheme(dtheta_step, rounds, model, 'frozen');
%! pos_port = robot_kinematics(rounds * dtheta_step, model);
%! assert(pos_port, pos_original, 1e-4);

%!test
%! % ... and the size of that first order error really is proportional to the
%! % step size, which is why the port is the more accurate of the two
%! model = pa10_model();
%! direction = [1, -1, 0.7, -0.5, 0.9, -0.8, 0.3];
%! coarse = 1e-2 * direction;
%! fine = 1e-3 * direction;
%! err_coarse = max(max(abs(original_matlab_scheme(coarse, 100, model, 'frozen') ...
%!                        - robot_kinematics(100 * coarse, model))));
%! err_fine = max(max(abs(original_matlab_scheme(fine, 1000, model, 'frozen') ...
%!                      - robot_kinematics(1000 * fine, model))));
%! assert(err_fine < 0.2 * err_coarse);

%!test
%! % and the zero configuration must be identical
%! model = pa10_model();
%! [pos_original, ~] = original_matlab_scheme(zeros(1, 7), 5, model);
%! pos_port = robot_kinematics(zeros(1, 7), model);
%! assert(pos_port, pos_original, 1e-12);
