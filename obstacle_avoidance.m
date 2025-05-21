clear; clc;
n = 7;
o = zeros(3,1);
x = zeros(3,n); x(1,:) = ones(1,n);
y = zeros(3,n); y(2,:) = ones(1,n);
z = zeros(3,n); z(3,:) = ones(1,n);

H = zeros(6*n,n);
link_dimensions = [.1135*z(:,1) ...
                    .307*x(:,2) ...
                    .143*x(:,3) ... 
                    .255*z(:,4) ...
                    .245*z(:,5) ...
                    .08*x(:,6) o];
rotation_axes = [z(:,1) ...
                 y(:,2) ...
                 x(:,3) ...
                 y(:,4) ...
                 z(:,5) ...
                 y(:,6) ...
                 x(:,7)];
link_colors=['b' 'b' 'r' 'r' 'g' 'g' 'w' 'w' 'g'];

cumulative_link_positions = [[0; 0; 0.1], cumsum(link_dimensions, 2)];
phi = eye(6*n);
for i = 1:n
   phi = phi + diag(ones(6*(n-i),1),-6*i);
   links(i) = Link(cumulative_link_positions(:, i), ...
                   cumulative_link_positions(:, i+1) + ...
                   cumulative_link_positions(:, i),link_colors(i+1));
end
phi_t=zeros(6,6*n); phi_t(:,6*n+(-5:0))=eye(6);

axis_max=.9;
axis([-axis_max axis_max -axis_max axis_max -axis_max axis_max]);
axis off
set(gcf, 'color', [0 0 0]);
hold on

Vt=zeros(6,1);
dtheta=0.07*[rand rand rand rand rand rand rand]';
for counter = 1:130
    for i = 1:n
        H(6*i+(-5:-3),i) = rotation_axes(:,i);
    end

    for i = 1:n
        phi(6*i+(-2:0),6*i+(-5:-3)) = SkewSymmetric(link_dimensions(:,n));
    end
    for i = 3:n+1
        for j = 1:i-2
            phi(6*i+(-8:-6),6*j+(-5:-3)) = phi(6*i+(-8:-6),6*i+(-11:-9)) + phi(6*i+(-14:-12),6*j+(-5:-3));
        end
    end

    phi_t(4:6,6*n+(-5:-3)) = SkewSymmetric(link_dimensions(:,n));

    J=phi_t*phi*H;

    cla
    p = [-.1 0 0; .1 0 0];
    plot3(p(:,1), p(:,2), p(:,3), link_colors(1), 'LineWidth', 2);
    p1 = o';
    p2 = [0 0 .1];
    p = [p1;p2];
    plot3(p(:,1), p(:,2), p(:,3), link_colors(1), 'LineWidth', 2);
    p1 = p2;
    for i = 1:n
        p2 = p1 + link_dimensions(:,i)';
        p = [p1;p2];
        plot3(p(:,1), p(:,2), p(:,3), link_colors(i), 'LineWidth', 6);
        if (i < n-1)
            plot3(p2(1), p2(2), p2(3), [link_colors(i) 'o'], 'LineWidth', 4);
        end
        p1 = p2;
    end
    k1 = .03;
    k2 = .02;
    p1 = p1 - k1*y(:,n)';
    p2 = p2 + k1*y(:,n)';
    p = [p1;p2];
    plot3(p(:,1), p(:,2), p(:,3), link_colors(7), 'LineWidth', 3);
    p3 = p1 + k2*x(:,n)';
    p = [p1;p3];
    plot3(p(:,1), p(:,2), p(:,3), link_colors(7), 'LineWidth', 3);
    p3 = p2 + k2*x(:,n)';
    p = [p2;p3];
    plot3(p(:,1), p(:,2), p(:,3), link_colors(7), 'LineWidth', 3);
    drawnow;

    if counter>10
        Vt(1) = -0.005;
        dtheta = pinv(J)*Vt;
    end

    for i = 1:n
        k = rotation_axes(:,i);
        R = RotateBy(k,dtheta(i));
        x(:,i:n) = R*x(:,i:n);
        y(:,i:n) = R*y(:,i:n);
        z(:,i:n) = R*z(:,i:n);
    end

    obstacle_pos = [0.3, 0.3, 0.3];
    
end