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

H = zeros(6*n, n);
link_colors = ['b', 'r', 'r', 'g', 'g', 'w', 'w', 'g'];

obstacle_pos = [0.3, 0.3, 0.3];
safety_parameter = 0.1;
obstacle_link = Link(obstacle_pos, ...
    obstacle_pos, ...
    x, ...
    'r', ...
    1, ...
    'o', ...
    safety_parameter, ...
    'r');

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
    obstacle_link.PlotJoints
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
            rotation_axes(i), ...
            link_colors(i));
        links(i).joint_color_ = link_colors(i);
        links(i).PlotJoints()
        links(i).PlotLink()
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
    p1 = [0, 0, .1];
    p2 = p1;
    for i = 1:n
        p2 = p1 + link_dimensions(:, i)';
        links(i).start_point_ = p1;
        links(i).end_point_ = p2;
        links(i).PlotLink
        links(i).PlotJoints
        p1 = p2;
    end
    drawnow;

    if counter > 10
        Vt(1) = -0.005;
        dtheta = pinv(J) * Vt;
    end

    for i = 1:n
        distance_to_obstacle = LineToPointDistance(links(i).end_point_, ...
            links(i).start_point_, ...
            obstacle_pos);
        if (distance_to_obstacle > safety_parameter)
            display("Link "+i+" Distance to Safety Zone: "+distance_to_obstacle)
            k = rotation_axes(:, i);
            R = RotateBy(k, dtheta(i));
            x(:, i:n) = R * x(:, i:n);
            y(:, i:n) = R * y(:, i:n);
            z(:, i:n) = R * z(:, i:n);
        else
            display("Link "+i+" is inside the unsafe perimeter.")
            display("Link "+i+" Position: X "+links(i).start_point_(1))
            display("Link "+i+" Position: Y "+links(i).start_point_(2))
            display("Link "+i+" Position: Z "+links(i).start_point_(3))
        end
    end

end
