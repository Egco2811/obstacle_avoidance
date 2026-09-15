function model = pa10_model()
%PA10_MODEL  Kinematic description of the 7-DOF PA-10 based demo arm.
%
%   MODEL = pa10_model () returns the kinematic parameters of the manipulator
%   used by the obstacle avoidance demos.  The numbers are the ones from the
%   original MATLAB proof of concept, so the arm keeps its geometry, its
%   proportions and its joint layout.
%
%   Model conventions
%   -----------------
%   Joint k rotates about the local unit axis JOINT_AXES(:,k), link k extends
%   along the local unit axis LINK_AXES(:,k) and has length LINK_LENGTHS(k).
%   "Local" means the frame obtained after applying the rotations of joints
%   1..k-1, therefore
%
%       C{0} = I
%       C{k} = C{k-1} * Rot(joint_axes(:,k), theta(k))
%       d{k} = C{k} * link_axes(:,k)            % link k direction, base frame
%       p{k} = p{k-1} + link_lengths(k) * d{k}  % link k end point, base frame
%
%   with p{0} = BASE_OFFSET.  ROBOT_KINEMATICS implements exactly this.
%
%   Note that the seventh link has zero length, which makes the seventh joint
%   a pure wrist roll: it re-orients the gripper but cannot move the tool
%   centre point.  This matches the original demo and is why the tool Jacobian
%   has a zero linear part in its last column.
%
%   Fields
%   ------
%     name          descriptive name
%     n             number of joints
%     base_offset   3x1 position of the first joint in the base frame [m]
%     joint_axes    3xN local rotation axes (unit vectors)
%     link_axes     3xN local link directions (unit vectors)
%     link_lengths  1xN link lengths [m]
%     link_colors   Nx3 RGB colours used to draw the links
%     joint_colors  Nx3 RGB colours used to draw the joints
%     base_half_width half width of the base pedestal [m]
%     gripper_offset  sideways offset of the gripper fingers [m]
%     gripper_length  length of the gripper fingers [m]
%     reach           maximum distance from the base to the tool point [m]
%
%   See also ROBOT_KINEMATICS, ROBOT_JACOBIAN, AVOIDANCE_CONTROLLER.

ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

model.name = 'PA-10 based 7-DOF manipulator';
model.n = 7;
model.base_offset = [0; 0; 0.1];

model.joint_axes = [ez, ey, ex, ey, ez, ey, ex];
model.link_axes = [ez, ex, ex, ez, ez, ex, ex];
model.link_lengths = [0.1135, 0.307, 0.143, 0.255, 0.245, 0.08, 0.0];

% Vivid but dark enough to stay readable on the white background that the
% Octave FLTK print path produces, and distinct enough from each other that
% neighbouring links are always easy to tell apart.
model.link_colors = [ ...
    0.11 0.45 0.82; ...   % blue
    0.09 0.60 0.34; ...   % green
    0.85 0.62 0.05; ...   % amber
    0.90 0.45 0.10; ...   % orange
    0.78 0.18 0.25; ...   % red
    0.53 0.28 0.78; ...   % violet
    0.10 0.58 0.68];      % teal
model.joint_colors = 0.72 * model.link_colors;

model.base_half_width = 0.10;
model.gripper_offset = 0.05;
model.gripper_length = 0.08;

model.reach = sum(model.link_lengths) + norm(model.base_offset);

end
