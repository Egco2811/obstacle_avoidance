classdef RobotScene < handle
%ROBOTSCENE  Animated 3-D view of the arm, the obstacles and the controller.
%
%   SCENE = RobotScene (MODEL, SCENARIO) opens a 3-D view of the manipulator.
%   SCENE.DRAW (THETA, INFO, STEP, N_STEPS, ELAPSED) then
%   refreshes the picture for one simulation step, where INFO is the output of
%   AVOIDANCE_CONTROLLER.
%
%   The default theme is light on purpose.  The FLTK toolkit, which is what a
%   plain Octave install offers, always renders PRINT output on a white
%   background regardless of the figure and axes colours, so a dark on-screen
%   theme would export unreadable frames.  The BACKGROUND and FOREGROUND
%   options still work for interactive use or for toolkits such as QT that do
%   honour them, but be aware that exported PNGs will come out white.
%
%   Two further FLTK quirks shape the drawing code, and both were confirmed by
%   rendering test frames rather than assumed:
%
%     * surfaces with FaceAlpha below about 0.5 are rendered wrongly.  The
%       safety shells are therefore drawn as wireframe cages with opaque edges
%       instead of as translucent bubbles.
%     * the mesh returned by Octave's own SPHERE function does not render at
%       all in this path, so SPHERE_SURFACE builds the sphere mesh instead.
%
%   Every graphics object is created once in the constructor and afterwards
%   only its XData/YData/ZData properties are refreshed.  Re-plotting the whole
%   scene on every frame is what made the original script crawl, and it is why
%   this class exists at all.
%
%   What is on screen
%     arm           one polyline per link, one marker per joint, in the
%                   per-joint colour palette from PA10_MODEL
%     gripper       the four finger segments, drawn in the last link frame
%     pedestal      the rounded base the arm is mounted on
%     obstacles     an opaque core plus a translucent wireframe safety shell,
%                   so the margin the controller actually reacts to is visible
%     tool trail    where the tool centre point has been
%     velocity      an arrow for the achieved tool velocity, and a second one
%                   for the escape velocity when a violation is being resolved
%     critical      the closest point of the arm to the nearest obstacle, tied
%                   to that obstacle by a dashed line and recoloured from cyan
%                   to red while the safety margin is violated
%     goal          the target tool position and the straight line to it
%     status panel  step, time, clearance and controller mode
%
%   Options (struct, all optional)
%     visible         show the window (default true)
%     figure          reuse an existing figure instead of creating one
%     toolkit         e.g. 'qt' or 'fltk'; empty keeps the current toolkit
%     title           headline, drawn above the box
%     subtitle        second line of the headline
%     window_title    window caption, defaults to TITLE
%     workspace       1x6 axis limits
%     view            1x2 [azimuth, elevation] in degrees
%     goal            3x1 tool target, drawn when non-empty
%     background      figure and axes colour
%     foreground      axis, label and text colour
%     grid_color      grid colour
%     trail_length    number of tool positions kept in the trail
%     draw_trail, draw_velocity, draw_safety_shells, draw_ground, draw_hud,
%     lighting        feature switches, all default true
%     link_width, joint_size, trail_width   line weights
%
%   See also SIMULATE_AVOIDANCE, AVOIDANCE_CONTROLLER, LINK.

    properties (SetAccess = private)
        figure_
        axes_
    end

    properties (Access = private)
        model_
        options_
        link_objects_ = {}
        gripper_objects_ = {}
        tip_marker_ = []
        obstacles_ = []
        trail_handle_ = []
        tool_path_ = []
        velocity_handle_ = []
        escape_handle_ = []
        critical_marker_ = []
        critical_line_ = []
        goal_marker_ = []
        goal_line_ = []
        hud_handle_ = []
        status_handle_ = []
    end

    methods
        function obj = RobotScene(model, scenario, options)
            obj.model_ = model;
            obj.options_ = scene_defaults(options, scenario);

            if ~isempty(obj.options_.toolkit)
                graphics_toolkit(obj.options_.toolkit);
            end

            if ~isempty(obj.options_.figure)
                obj.figure_ = obj.options_.figure;
            elseif obj.options_.visible
                visibility = 'on';
                obj.figure_ = figure('Visible', visibility, 'Position', [80, 80, 900, 760], ...
                    'Color', obj.options_.background, ...
                    'Name', obj.options_.window_title, 'NumberTitle', 'off');
            else
                obj.figure_ = figure('Visible', 'off', 'Position', [80, 80, 900, 760], ...
                    'Color', obj.options_.background, ...
                    'Name', obj.options_.window_title, 'NumberTitle', 'off');
            end
            % Print the figure at the size it is on screen.  Without this the
            % FLTK print path falls back to the paper size and the exported
            % frames come out in a different aspect ratio than the window.
            set(obj.figure_, 'PaperPositionMode', 'auto');

            obj.axes_ = axes('Parent', obj.figure_, ...
                'Color', obj.options_.background, ...
                'Position', [0.07, 0.19, 0.89, 0.68]);
            hold(obj.axes_, 'on');
            obj.SetupAxes(scenario);

            if obj.options_.draw_ground
                obj.DrawGround();
            end
            obj.DrawPedestal();
            obj.SetupArm();
            obj.SetupGripper();

            if obj.options_.draw_trail
                obj.trail_handle_ = line('Parent', obj.axes_, ...
                    'XData', [], 'YData', [], 'ZData', [], ...
                    'Color', [0.95, 0.52, 0.08], 'LineWidth', obj.options_.trail_width);
            end

            if obj.options_.draw_velocity
                obj.velocity_handle_ = quiver3(obj.axes_, 0, 0, 0, 0, 0, 0, ...
                    'Color', [0.10, 0.42, 0.85], 'LineWidth', 1.5, 'MaxHeadSize', 0.12);
                obj.escape_handle_ = quiver3(obj.axes_, 0, 0, 0, 0, 0, 0, ...
                    'Color', [0.88, 0.16, 0.16], 'LineWidth', 1.5, 'MaxHeadSize', 0.12);
            end

            obj.critical_marker_ = line('Parent', obj.axes_, ...
                'XData', 0, 'YData', 0, 'ZData', 0, ...
                'Marker', 'o', 'MarkerSize', 8, 'LineStyle', 'none', ...
                'MarkerFaceColor', [0.05, 0.60, 0.55], 'Color', [0.05, 0.60, 0.55]);
            obj.critical_line_ = line('Parent', obj.axes_, ...
                'XData', [0, 0], 'YData', [0, 0], 'ZData', [0, 0], ...
                'Color', [0.05, 0.60, 0.55], 'LineStyle', '--', 'LineWidth', 1.2);

            obj.SetupObstacles(scenario.obstacles);
            obj.SetupGoal();

            if obj.options_.draw_hud
                obj.status_handle_ = text('Parent', obj.axes_, 0.0, -0.055, 0, '', ...
                    'Units', 'normalized', 'Color', [0.06, 0.50, 0.22], ...
                    'FontName', 'Helvetica', 'FontSize', 9, 'FontWeight', 'bold', ...
                    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
                    'Clipping', 'off', 'Interpreter', 'none');
                obj.hud_handle_ = text('Parent', obj.axes_, 0.0, -0.14, 0, {''}, ...
                    'Units', 'normalized', 'Color', obj.options_.foreground, ...
                    'FontName', 'Helvetica', 'FontSize', 8, ...
                    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
                    'Clipping', 'off', 'Interpreter', 'none');
            end

            if obj.options_.lighting
                try
                    light('Parent', obj.axes_, 'Position', [1.2, -1.0, 1.6], 'Style', 'local');
                    light('Parent', obj.axes_, 'Position', [-1.4, 1.2, 0.8], 'Style', 'local');
                    lighting(obj.axes_, 'gouraud');
                    material(obj.axes_, 'dull');
                catch
                    % Lighting is cosmetic: a toolkit without lights still
                    % renders the scene, it just looks flatter.
                end
            end
        end

        function delete(obj)
            %DELETE  Close the window when the scene goes out of scope.
            if ~isempty(obj.figure_) && ishandle(obj.figure_)
                delete(obj.figure_);
            end
        end

        function draw(obj, theta, info, step, n_steps, elapsed)
            %DRAW  Refresh the whole scene for one simulation step.
            obj.UpdateArm(info.pos, info.frames);
            obj.UpdateObstacles(info.clearance, info.resolved_obstacles);
            obj.UpdateTrail(info.pos(:, end));
            obj.UpdateVelocity(info, step);
            obj.UpdateCritical(info);
            obj.UpdateHud(info, step, n_steps, elapsed);
            drawnow();
        end

        function save_frame(obj, filename)
            %SAVE_FRAME  Write the current view to a PNG file.
            print(obj.figure_, filename, '-dpng', '-r100');
        end
    end

    methods (Access = private)
        function SetupAxes(obj, scenario)
            ax = obj.axes_;
            limits = obj.options_.workspace;

            set(ax, 'Color', obj.options_.background, ...
                'XColor', obj.options_.foreground, ...
                'YColor', obj.options_.foreground, ...
                'ZColor', obj.options_.foreground, ...
                'GridColor', obj.options_.grid_color, ...
                'GridAlpha', 0.35, ...
                'Box', 'on', 'BoxStyle', 'full', ...
                'FontSize', 8);
            try
                set(ax, 'GridLineStyle', ':');
            catch
            end

            xlabel(ax, 'x [m]');
            ylabel(ax, 'y [m]');
            zlabel(ax, 'z [m]');

            axis(ax, limits);
            axis(ax, 'manual');
            set(ax, 'DataAspectRatio', [1, 1, 1], ...
                'DataAspectRatioMode', 'manual', ...
                'PlotBoxAspectRatioMode', 'auto');
            grid(ax, 'on');

            if ~isempty(obj.options_.view)
                view(ax, obj.options_.view(1), obj.options_.view(2));
            end
            if ~isempty(obj.options_.title)
                % Kept to a single line: the FLTK print path drops the second
                % line of a multi-line title.
                headline = obj.options_.title;
                if ~isempty(obj.options_.subtitle)
                    headline = sprintf('%s  |  %s', headline, obj.options_.subtitle);
                end
                title(ax, headline, 'Color', obj.options_.foreground, ...
                    'FontSize', 10, 'FontWeight', 'bold');
            end
            try
                rotate3d(ax, 'off');
            catch
                % rotate3d needs Java, which a headless Octave usually lacks.
            end
        end

        function DrawGround(obj)
            limits = obj.options_.workspace;
            x = [limits(1); limits(2); limits(2); limits(1)];
            y = [limits(3); limits(3); limits(4); limits(4)];
            z = limits(5) * ones(4, 1);
            patch('Parent', obj.axes_, 'XData', x, 'YData', y, 'ZData', z, ...
                'FaceColor', [0.94, 0.95, 0.97], ...
                'EdgeColor', [0.78, 0.81, 0.86], 'LineWidth', 0.5);
        end

        function DrawPedestal(obj)
            [X, Y, Z] = capsule_surface([0; 0; -0.02], [0; 0; 0.06], 0.085);
            surf('Parent', obj.axes_, 'XData', X, 'YData', Y, 'ZData', Z, ...
                'FaceColor', [0.62, 0.65, 0.71], 'EdgeColor', 'none');
        end

        function SetupArm(obj)
            model = obj.model_;
            for k = 1:model.n
                link = Link(zeros(3, 1), zeros(3, 1), model.joint_axes(:, k), ...
                    model.link_colors(k, :), obj.options_.link_width, ...
                    'o', obj.options_.joint_size, model.joint_colors(k, :));
                link.draw_start_joint_ = true;
                link.draw_end_joint_ = false;
                link.Draw(obj.axes_);
                obj.link_objects_{k} = link;
            end

            obj.tip_marker_ = line('Parent', obj.axes_, ...
                'XData', 0, 'YData', 0, 'ZData', 0, ...
                'Marker', 'o', 'MarkerSize', obj.options_.joint_size + 2, ...
                'MarkerFaceColor', [0.12, 0.15, 0.20], 'Color', [0.12, 0.15, 0.20], ...
                'LineStyle', 'none');
        end

        function SetupGripper(obj)
            for k = 1:4
                link = Link(zeros(3, 1), zeros(3, 1), [1; 0; 0], ...
                    [0.28, 0.31, 0.38], max(2, obj.options_.link_width - 4), ...
                    'o', 4, [0.28, 0.31, 0.38]);
                link.draw_start_joint_ = false;
                link.Draw(obj.axes_);
                obj.gripper_objects_{k} = link;
            end
        end

        function SetupObstacles(obj, obstacles)
            %SETUPOBSTACLES  Create a core, a shell and a floor stem per obstacle.
            if isempty(obstacles)
                obj.obstacles_ = [];
                return;
            end

            obj.obstacles_ = struct('core', {}, 'shell', {}, 'stem', {});
            for j = 1:numel(obstacles)
                obstacle = obstacles(j);
                safety = obstacle.safety;
                if isempty(safety)
                    safety = 0.05;
                end
                outer = obstacle.radius + safety;

                [X, Y, Z] = obj.SurfaceFor(obstacle, obstacle.radius);
                obj.obstacles_(j).core = surf('Parent', obj.axes_, ...
                    'XData', X, 'YData', Y, 'ZData', Z, ...
                    'FaceColor', obstacle.color, 'EdgeColor', 'none');

                if obj.options_.draw_safety_shells
                    [X, Y, Z] = obj.ShellSurfaceFor(obstacle, outer);
                    obj.obstacles_(j).shell = surf('Parent', obj.axes_, ...
                        'XData', X, 'YData', Y, 'ZData', Z, ...
                        'FaceColor', 'none', ...
                        'EdgeColor', obstacle.color, 'LineWidth', 0.5);
                else
                    obj.obstacles_(j).shell = [];
                end

                obj.obstacles_(j).stem = line('Parent', obj.axes_, ...
                    'XData', [0, 0], 'YData', [0, 0], 'ZData', [0, 0], ...
                    'Color', obj.options_.grid_color, 'LineStyle', '-', ...
                    'LineWidth', 0.75);
            end
        end

        function [X, Y, Z] = SurfaceFor(obj, obstacle, radius)
            %SURFACEFOR  Obstacle mesh, drawn as an opaque body.
            if strcmpi(obstacle.type, 'sphere')
                [X, Y, Z] = sphere_surface(obstacle.center, radius, 12, 24);
            else
                [X, Y, Z] = capsule_surface(obstacle.p1, obstacle.p2, radius, 12, 24);
            end
        end

        function [X, Y, Z] = ShellSurfaceFor(obj, obstacle, radius)
            %SHELLSURFACEFOR  Coarse safety shell, drawn as a wireframe cage.
            %
            %   A coarse mesh is deliberate: the shell is there to show how much
            %   room the controller is keeping, and a dense mesh would just look
            %   like a solid blob.
            if strcmpi(obstacle.type, 'sphere')
                [X, Y, Z] = sphere_surface(obstacle.center, radius, 5, 12);
            else
                [X, Y, Z] = capsule_surface(obstacle.p1, obstacle.p2, radius, 4, 10);
            end
        end

        function SetupGoal(obj)
            goal = obj.options_.goal;
            if isempty(goal)
                return;
            end
            goal = goal(:);
            obj.goal_marker_ = line('Parent', obj.axes_, ...
                'XData', goal(1), 'YData', goal(2), 'ZData', goal(3), ...
                'Marker', 'p', 'MarkerSize', 16, 'LineStyle', 'none', ...
                'MarkerFaceColor', obj.options_.goal_color, ...
                'Color', obj.options_.goal_color);

            limits = obj.options_.workspace;
            obj.goal_line_ = line('Parent', obj.axes_, ...
                'XData', [goal(1), goal(1)], ...
                'YData', [goal(2), goal(2)], ...
                'ZData', [limits(5), goal(3)], ...
                'Color', obj.options_.goal_color, 'LineStyle', ':', ...
                'LineWidth', 0.75);
        end

        function UpdateArm(obj, pos, frames)
            model = obj.model_;
            for k = 1:model.n
                obj.link_objects_{k}.SetPoints(pos(:, k), pos(:, k + 1));
                obj.link_objects_{k}.Update();
            end
            set(obj.tip_marker_, ...
                'XData', pos(1, end), 'YData', pos(2, end), 'ZData', pos(3, end));

            % The gripper lives in the frame of the last link, like the
            % actuator links of the original demo.
            grip_frame = frames(:, :, end);
            tip = pos(:, end);
            sideways = grip_frame * [0; 1; 0];
            forward = grip_frame * [1; 0; 0];
            finger_a = tip - model.gripper_offset * sideways;
            finger_b = tip + model.gripper_offset * sideways;

            points = { ...
                tip, finger_a; ...
                tip, finger_b; ...
                finger_a, finger_a + model.gripper_length * forward; ...
                finger_b, finger_b + model.gripper_length * forward};
            for k = 1:4
                obj.gripper_objects_{k}.SetPoints(points{k, 1}, points{k, 2});
                obj.gripper_objects_{k}.Update();
            end
        end

        function UpdateObstacles(obj, clearance, resolved)
            if isempty(obj.obstacles_)
                return;
            end
            limits = obj.options_.workspace;
            for j = 1:numel(obj.obstacles_)
                obstacle = resolved(j);
                safety = clearance(j).margin;
                if isempty(safety)
                    safety = 0.05;
                end

                [X, Y, Z] = obj.SurfaceFor(obstacle, obstacle.radius);
                set(obj.obstacles_(j).core, 'XData', X, 'YData', Y, 'ZData', Z);

                if ~isempty(obj.obstacles_(j).shell)
                    [X, Y, Z] = obj.ShellSurfaceFor(obstacle, obstacle.radius + safety);
                    set(obj.obstacles_(j).shell, 'XData', X, 'YData', Y, 'ZData', Z);
                end

                anchor = obstacle.center(:);
                if isempty(anchor) && ~isempty(obstacle.p1)
                    anchor = 0.5 * (obstacle.p1(:) + obstacle.p2(:));
                end
                set(obj.obstacles_(j).stem, ...
                    'XData', [anchor(1), anchor(1)], ...
                    'YData', [anchor(2), anchor(2)], ...
                    'ZData', [limits(5), anchor(3)]);
            end
        end

        function UpdateTrail(obj, tool_point)
            if isempty(obj.trail_handle_)
                return;
            end
            obj.tool_path_ = [obj.tool_path_, tool_point];
            if size(obj.tool_path_, 2) > obj.options_.trail_length
                obj.tool_path_ = obj.tool_path_(:, end - obj.options_.trail_length + 1:end);
            end
            set(obj.trail_handle_, ...
                'XData', obj.tool_path_(1, :), ...
                'YData', obj.tool_path_(2, :), ...
                'ZData', obj.tool_path_(3, :));
        end

        function UpdateVelocity(obj, info, step)
            if isempty(obj.velocity_handle_)
                return;
            end
            tip = info.pos(:, end);
            scale = obj.options_.velocity_scale;

            if isfield(info, 'tip_velocity')
                velocity = scale * info.tip_velocity(1:3);
                set(obj.velocity_handle_, 'XData', tip(1), 'YData', tip(2), 'ZData', tip(3), ...
                    'UData', velocity(1), 'VData', velocity(2), 'WData', velocity(3));
            end

            if ~isempty(obj.escape_handle_)
                if isfield(info, 'escape_vector') && norm(info.escape_vector) > 0
                    point = info.clearance(info.active_index).robot_point;
                    escape = scale * info.escape_vector;
                    set(obj.escape_handle_, ...
                        'XData', point(1), 'YData', point(2), 'ZData', point(3), ...
                        'UData', escape(1), 'VData', escape(2), 'WData', escape(3));
                else
                    set(obj.escape_handle_, 'UData', 0, 'VData', 0, 'WData', 0);
                end
            end
        end

        function UpdateCritical(obj, info)
            if isempty(info.clearance)
                return;
            end
            nearest = info.nearest_index;
            entry = info.clearance(nearest);

            if info.violation(nearest) > 0
                color = [0.85, 0.12, 0.12];
            else
                color = [0.05, 0.60, 0.55];
            end

            robot_point = entry.robot_point;
            obstacle_point = entry.obstacle_point;
            set(obj.critical_marker_, 'XData', robot_point(1), 'YData', robot_point(2), ...
                'ZData', robot_point(3), 'MarkerFaceColor', color, 'Color', color);
            set(obj.critical_line_, 'XData', [robot_point(1), obstacle_point(1)], ...
                'YData', [robot_point(2), obstacle_point(2)], ...
                'ZData', [robot_point(3), obstacle_point(3)], 'Color', color);
        end

        function UpdateHud(obj, info, step, n_steps, elapsed)
            if isempty(obj.hud_handle_)
                return;
            end

            clearance = [info.clearance.clearance];
            [minimum, nearest] = min(clearance);
            margin = info.clearance(nearest).margin;

            lines = { ...
                sprintf('step %d / %d', step, n_steps), ...
                sprintf('t = %.1f s', elapsed), ...
                sprintf('tool speed  %.3f m/s', norm(info.tip_velocity(1:3))), ...
                sprintf('clearance   %+.3f m  (margin %.3f m)', minimum, margin), ...
                sprintf('tool xyz    %+.2f  %+.2f  %+.2f', info.pos(:, end))};
            set(obj.hud_handle_, 'String', lines);

            if info.active
                set(obj.status_handle_, 'String', ...
                    sprintf('AVOIDING  |  %s  |  escape %.3f m/s', ...
                            info.mode, info.escape_speed), ...
                    'Color', [0.82, 0.14, 0.14]);
            else
                set(obj.status_handle_, 'String', 'CLEAR  |  tracking goal', ...
                    'Color', [0.06, 0.50, 0.22]);
            end
        end
    end
end

function options = scene_defaults(options, scenario)
%SCENE_DEFAULTS  Fill in missing scene options.

if nargin < 1 || isempty(options)
    options = struct();
end
if nargin < 2 || isempty(scenario)
    scenario = struct();
end

defaults = struct( ...
    'visible', true, ...
    'figure', [], ...
    'toolkit', '', ...
    'title', 'Obstacle avoidance', ...
    'subtitle', '', ...
    'window_title', '', ...
    'workspace', [-0.6, 0.6, -0.6, 0.6, -0.1, 1.1], ...
    'view', [-40, 22], ...
    'goal', [], ...
    'background', [1.0, 1.0, 1.0], ...
    'foreground', [0.20, 0.24, 0.30], ...
    'grid_color', [0.72, 0.76, 0.82], ...
    'goal_color', [0.10, 0.60, 0.28], ...
    'trail_length', 2000, ...
    'trail_width', 1.6, ...
    'link_width', 7, ...
    'joint_size', 8, ...
    'velocity_scale', 1.5, ...
    'draw_trail', true, ...
    'draw_velocity', true, ...
    'draw_safety_shells', true, ...
    'draw_ground', true, ...
    'draw_hud', true, ...
    'lighting', true);

names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(options, name) || isempty(options.(name))
        options.(name) = defaults.(name);
    end
end

if isfield(scenario, 'goal') && isempty(options.goal)
    options.goal = scenario.goal;
end
if isempty(options.window_title)
    options.window_title = options.title;
end
if isfield(scenario, 'workspace') && ~isempty(scenario.workspace)
    options.workspace = scenario.workspace;
end
if isfield(scenario, 'camera') && ~isempty(scenario.camera)
    options.view = scenario.camera;
end

end
