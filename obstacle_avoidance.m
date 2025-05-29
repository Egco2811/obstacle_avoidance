clear;
clc;
n = 7;
o = zeros(3, 1);
x = zeros(3, n);
x(1, :) = ones(1, n);
y = zeros(3, n);
y(2, :) = ones(1, n);
z = zeros(3, n);
z(3, :) = ones(1, n);
link_flags = zeros(n); 

H = zeros(6*n, n);
link_colors = ['b', 'r', 'r', 'g', 'g', 'w', 'w', 'g'];

obstacle_pos = [0.2, 0.2, 0.4];
safety_perimeter = 0.1;
[radius_x, radius_y, radius_z] = sphere();
obstacle_x = radius_x * safety_perimeter + obstacle_pos(1);
obstacle_y = radius_y * safety_perimeter + obstacle_pos(2);
obstacle_z = radius_z * safety_perimeter + obstacle_pos(3);

base_axis_placeholder = [1;0;0]; 
base_link = Link([-.1, 0, 0], [.1, 0, 0], base_axis_placeholder, link_colors(1));
base_link_column = Link(o, [0, 0, .1], base_axis_placeholder, link_colors(1));

phi = eye(6*n);

axis_max = .9;
axis([-axis_max, axis_max, -axis_max, axis_max, -axis_max, axis_max]);
axis off
set(gcf, 'color', [0, 0, 0]);
hold on

links = Link.empty(0, n);
for i = 1:n
    phi = phi + diag(ones(6*(n - i), 1), -6*i);
end
%%
phi_t = zeros(6, 6*n);
phi_t(:, 6*n+(-5:0)) = eye(6);

Vt = zeros(6, 1);
dtheta = 0.07 * [rand, rand, rand, rand, rand, rand, rand]';
for counter = 1:130
    link_dimensions = [[0; 0; 0.1], ...
        .1135 * z(:, 1), ...
        .307 * x(:, 2), ...
        .143 * x(:, 3), ...
        .255 * z(:, 4), ...
        .245 * z(:, 5), ...
        .08 * x(:, 6), o];
    rotation_axes = [z(:, 1), ...
        y(:, 2), ...
        x(:, 3), ...
        y(:, 4), ...
        z(:, 5), ...
        y(:, 6), ...
        x(:, 7)];
    cumulative_link_positions = cumsum(link_dimensions, 2);

    for i = 1:n
        links(i) = Link(cumulative_link_positions(:, i), ...
            cumulative_link_positions(:, i+1), ...
            rotation_axes(:,i), ... 
            link_colors(i));
        links(i).joint_color_ = link_colors(i);
    end

    for i = 1:n
        H(6*i+(-5:-3), i) = rotation_axes(:, i);
    end

    for i = 1:n 
        phi(6*i+(-2:0), 6*i+(-5:-3)) = SkewSymmetric(link_dimensions(:, n));
    end
    for i = 3:n + 1 
        for j = 1:i - 2
            phi(6*i+(-8:-6), 6*j+(-5:-3)) = phi(6*i+(-8:-6), 6*i+(-11:-9)) + phi(6*i+(-14:-12), 6*j+(-5:-3));
        end
    end

    phi_t(4:6, 6*n+(-5:-3)) = SkewSymmetric(link_dimensions(:, n)); 

    J = phi_t * phi * H;

    cla
    p1 = [0; 0; .1]; 
    p2 = p1;
    for i = 1:n
        p2 = p1 + link_dimensions(:, i); 
        links(i).start_point_ = p1;
        links(i).end_point_ = p2;
        links(i).PlotLink
        links(i).PlotJoints
        p1 = p2;
    end
    k1 = .05;
    k2 = .08;
    actuator_link_1 = Link(links(n).end_point_, links(n).end_point_-k1*y(:, n)); 
    actuator_link_2 = Link(links(n).end_point_, links(n).end_point_+k1*y(:, n));
    actuator_link_3 = Link(actuator_link_1.end_point_, actuator_link_1.end_point_+k2*x(:, n)); 
    actuator_link_4 = Link(actuator_link_2.end_point_, actuator_link_2.end_point_+k2*x(:, n));
    base_link.PlotLink
    base_link_column.PlotLink
    actuator_link_1.PlotLink
    actuator_link_2.PlotLink
    actuator_link_3.PlotLink
    actuator_link_4.PlotLink
    surf(obstacle_x, obstacle_y, obstacle_z, ...
        'FaceColor', 'r', ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.4);
    drawnow;

    if counter > 10
        Vt(1) = -0.005;
        if rank(J) < min(size(J)) 
            dtheta = pinv(J) * Vt; 
        else
            dtheta = J \ Vt; 
        end
    end

    first_violating_link_index = -1;

    for check_idx = 1:n
        distance_to_obstacle = LineToPointDistance(links(check_idx).end_point_, ...
                                                  links(check_idx).start_point_, ...
                                                  obstacle_pos);
        if (distance_to_obstacle < safety_perimeter)
            first_violating_link_index = check_idx;
            display("Link "+check_idx+" is the first to violate safety perimeter. Distance: "+distance_to_obstacle);
            display("Link "+check_idx+" Start Position: X "+links(check_idx).start_point_(1)+...
                    " Y "+links(check_idx).start_point_(2)+...
                    " Z "+links(check_idx).start_point_(3));
            break;
        end
    end

    for i = 1:n
        apply_rotation_for_this_link = false;

        if first_violating_link_index == -1
            apply_rotation_for_this_link = true;
        else
            if i >= first_violating_link_index
                apply_rotation_for_this_link = true;
                if i == first_violating_link_index
                    display("Link "+i+": Is the first violator. Applying dtheta as per rule.");
                else
                    display("Link "+i+": Is after first violator ("+first_violating_link_index+"). Applying dtheta as per rule.");
                end
            else
                display("Link "+i+": Is before first violator ("+first_violating_link_index+"). NOT applying dtheta.");
            end
        end

        if apply_rotation_for_this_link
            if first_violating_link_index == -1
                 display("Link "+i+" Distance to Safety Zone (no violator): "+ LineToPointDistance(links(i).end_point_, links(i).start_point_, obstacle_pos));
            end
            k = rotation_axes(:, i);
            R = RotateBy(k, dtheta(i));
            x(:, i:n) = R * x(:, i:n);
            y(:, i:n) = R * y(:, i:n);
            z(:, i:n) = R * z(:, i:n);
        else
            %avoidance algorithm
        end
    end
end
