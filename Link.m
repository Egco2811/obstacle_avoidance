classdef Link
    properties
        start_point_
        end_point_
        link_color_
        line_width_
        joint_marker_
        joint_size_
        joint_color_
    end

    methods
        function obj = Link(start_point, end_point, link_color, line_width, ...
                joint_marker, joint_size, joint_color)
            if nargin < 1, start_point = [0, 0, 0]; end
            if nargin < 2, end_point = [0, 0, 0]; end
            if nargin < 3, link_color = 'b'; end
            if nargin < 4, line_width = 4; end
            if nargin < 5, joint_marker = 'o'; end
            if nargin < 6, joint_size = 6; end
            if nargin < 7, joint_color = 'b'; end
            obj.start_point_ = start_point;
            obj.end_point_ = end_point;
            obj.link_color_ = link_color;
            obj.line_width_ = line_width;
            obj.joint_marker_ = joint_marker;
            obj.joint_size_ = joint_size;
            obj.joint_color_ = joint_color;
        end
        function PlotLink(obj)
            plot3([obj.start_point_(1); obj.end_point_(1)], ...
                  [obj.start_point_(2); obj.end_point_(2)], ...
                  [obj.start_point_(3); obj.end_point_(3)], ...
                  obj.link_color_, 'LineWidth', obj.line_width_);
        end
        function PlotJoints(obj)
            plot3(obj.start_point_, obj.color_, ...
                  'LineWidth', obj.line_width_*2);
            plot3(obj.end_point_, obj.color_, ...
                  'LineWidth', obj.line_width_*2);
        end
    end
end