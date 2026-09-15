classdef Link < handle
%LINK  One drawn segment of the manipulator: a line with an optional joint.
%
%   The class is deliberately small.  Its job is to own the graphics handles of
%   a single segment so that an animation can refresh XData/YData/ZData on
%   every frame instead of deleting and re-plotting, which is what made the
%   original script slow.
%
%   Differences from the original MATLAB version:
%     * it derives from HANDLE, so Update() mutates the drawn object instead
%       of a copy (the original was a value class)
%     * Draw() creates the graphics objects once and Update() refreshes them
%     * the MATLAB-only static constructor Link.empty() is gone, because
%       Octave classdefs do not support it.  Build a cell array of links
%       instead:
%
%           links = cell(1, n);
%           for k = 1:n
%               links{k} = Link(start_point, end_point, axis, colour);
%               links{k}.Draw(ax);
%           end
%
%     * colours may be RGB triplets as well as the single letter MATLAB
%       colour codes the original used
%     * PlotLink() and PlotJoints() are kept for compatibility; they plot
%       immediately, exactly like the original methods did
%
%   Created by Egco 21/05/2025
%
%   See also ROBOTSCENE.

    properties
        start_point_
        end_point_
        rotation_axis_
        link_color_
        line_width_
        joint_marker_
        joint_size_
        joint_color_
        draw_start_joint_ = true
        draw_end_joint_ = false
        line_handle_ = []
        start_joint_handle_ = []
        end_joint_handle_ = []
    end

    methods
        function obj = Link(start_point, end_point, rotation_axis, ...
                            link_color, line_width, joint_marker, ...
                            joint_size, joint_color)
            if nargin < 1 || isempty(start_point), start_point = [0; 0; 0]; end
            if nargin < 2 || isempty(end_point), end_point = [0; 0; 0]; end
            if nargin < 3 || isempty(rotation_axis), rotation_axis = [0; 0; 0]; end
            if nargin < 4 || isempty(link_color), link_color = 'b'; end
            if nargin < 5 || isempty(line_width), line_width = 3; end
            if nargin < 6 || isempty(joint_marker), joint_marker = 'o'; end
            if nargin < 7 || isempty(joint_size), joint_size = 1; end
            if nargin < 8 || isempty(joint_color), joint_color = link_color; end

            obj.start_point_ = start_point(:);
            obj.end_point_ = end_point(:);
            obj.rotation_axis_ = rotation_axis(:);
            obj.link_color_ = link_color;
            obj.line_width_ = line_width;
            obj.joint_marker_ = joint_marker;
            obj.joint_size_ = joint_size;
            obj.joint_color_ = joint_color;
        end

        function SetPoints(obj, start_point, end_point)
            %SETPOINTS  Move the segment without touching the graphics.
            obj.start_point_ = start_point(:);
            obj.end_point_ = end_point(:);
        end

        function Draw(obj, ax)
            %DRAW  Create the graphics objects for this segment, once.
            if nargin < 2 || isempty(ax)
                ax = gca();
            end

            obj.line_handle_ = line('Parent', ax, ...
                'XData', [obj.start_point_(1); obj.end_point_(1)], ...
                'YData', [obj.start_point_(2); obj.end_point_(2)], ...
                'ZData', [obj.start_point_(3); obj.end_point_(3)], ...
                'Color', obj.link_color_, ...
                'LineWidth', obj.line_width_);

            if obj.draw_start_joint_
                obj.start_joint_handle_ = obj.MakeJoint(ax, obj.start_point_, obj.joint_color_);
            end
            if obj.draw_end_joint_
                obj.end_joint_handle_ = obj.MakeJoint(ax, obj.end_point_, obj.joint_color_);
            end
        end

        function Update(obj)
            %UPDATE  Refresh the graphics after SetPoints.
            if ~isempty(obj.line_handle_) && ishandle(obj.line_handle_)
                set(obj.line_handle_, ...
                    'XData', [obj.start_point_(1); obj.end_point_(1)], ...
                    'YData', [obj.start_point_(2); obj.end_point_(2)], ...
                    'ZData', [obj.start_point_(3); obj.end_point_(3)]);
            end
            if ~isempty(obj.start_joint_handle_) && ishandle(obj.start_joint_handle_)
                set(obj.start_joint_handle_, ...
                    'XData', obj.start_point_(1), ...
                    'YData', obj.start_point_(2), ...
                    'ZData', obj.start_point_(3));
            end
            if ~isempty(obj.end_joint_handle_) && ishandle(obj.end_joint_handle_)
                set(obj.end_joint_handle_, ...
                    'XData', obj.end_point_(1), ...
                    'YData', obj.end_point_(2), ...
                    'ZData', obj.end_point_(3));
            end
        end

        function PlotLink(obj)
            %PLOTLINK  Immediate one-off plot of the segment (original API).
            plot3([obj.start_point_(1); obj.end_point_(1)], ...
                  [obj.start_point_(2); obj.end_point_(2)], ...
                  [obj.start_point_(3); obj.end_point_(3)], ...
                  'Color', obj.link_color_, 'LineWidth', obj.line_width_);
        end

        function PlotJoints(obj)
            %PLOTJOINTS  Immediate one-off plot of both joint markers.
            points = [obj.start_point_, obj.end_point_];
            plot3(points(1, :), points(2, :), points(3, :), ...
                  'Marker', obj.joint_marker_, ...
                  'Color', obj.joint_color_, ...
                  'MarkerFaceColor', obj.joint_color_, ...
                  'LineStyle', 'none', ...
                  'MarkerSize', obj.joint_size_);
        end
    end

    methods (Access = private)
        function handle = MakeJoint(obj, ax, point, color)
            handle = line('Parent', ax, ...
                'XData', point(1), 'YData', point(2), 'ZData', point(3), ...
                'Marker', obj.joint_marker_, ...
                'MarkerSize', obj.joint_size_, ...
                'MarkerFaceColor', color, ...
                'Color', color, ...
                'LineStyle', 'none');
        end
    end
end
