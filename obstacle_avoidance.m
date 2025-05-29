clear 
n = 7;
o = zeros(3,1);
x = zeros(3,n); x(1,:) = ones(1,n);
y = zeros(3,n); y(2,:) = ones(1,n);
z = zeros(3,n); z(3,:) = ones(1,n);

H = zeros(6*n,n);

phi = eye(6*n);
for i = 1:n
   phi = phi + diag(ones(6*(n-i),1),-6*i);
end

phi_t=zeros(6,6*n); phi_t(:,6*n+(-5:0))=eye(6);


Vt=zeros(6,1);
dtheta=0.07*[rand rand rand rand rand rand rand]';
obstacle_pos = [0.4, 0.17, 0.25];
% obstacle_pos = [0.3, 0.2, 0.1];
% obstacle_pos = [0.4, 0.2, 0.45]
avoidance_radius = .1;
obstacle_avoidance_mode = true;

% target tip speed
Vt(4) = -0.001;
Vt(5) = 0.0015;
Vt(6) = -0.001;

conf = zeros(3,n,200);

for counter = 1:200
    L = [.1135*z(:,1)  .307*x(:,2)  .143*x(:,3)  .255*z(:,4)  .245*z(:,5)  .08*x(:,6) o];
    h = [z(:,1) y(:,2) x(:,3) y(:,4) z(:,5) y(:,6) x(:,7)];


    % % Doğru orta noktalar için
    joint_positions = zeros(3, n+1);
    joint_positions(:,1) = [0; 0; 0.1];

    for i = 1:n
        joint_positions(:,i+1) = joint_positions(:,i) + L(:,i);
    end

    link_distances = zeros(n, 1);
    closest_points = zeros(3, n);  % HER BİR BAĞLANTI İÇİN EN YAKIN NOKTAYI TUTACAĞIZ
    
    for i = 1:n
        p1 = joint_positions(:, i);
        p2 = joint_positions(:, i+1);
        v = p2 - p1;
        w = obstacle_pos' - p1;
    
        t = dot(w, v) / dot(v, v);
    
        if t >= 0 && t <= 1
            closest_point = p1 + t * v;
        elseif t < 0
            closest_point = p1;
        else
            closest_point = p2;
        end
    
        closest_points(:, i) = closest_point;  % işte burası önemli
        link_distances(i) = norm(obstacle_pos' - closest_point);
    end


    valid_distances = link_distances < avoidance_radius;
    if any(valid_distances)
        % Yalnızca eşik altındaki uzaklıklardan en küçüğünün indeksini bul
        [~, a] = min(link_distances .* valid_distances + (~valid_distances) * 1e6);
    else
        a = 0; % Engele yakın link yok
    end

    % a = 0;

    if a>0 && obstacle_avoidance_mode %obtacle avoidance mode
        disp(a)
        H1 = zeros(6*a,a);
        H2 = zeros(6*(n-a),n-a);
        phi_t1 = zeros(6,6*a); phi_t2 = zeros(6,6*(n-a));
        phi1 = eye(6*a); phi_t1(:,6*a+(-5:0))=eye(6);
        phi2 = eye(6*(n-a)); phi_t2(:,6*(n-a)+(-5:0))=eye(6);
        L1 = L(:,1:a); L2 = L(:,a+1:n);
        h1 = h(:,1:a); h2 = h(:,a+1:n);

        for i = 1:a
            H1(6*i+(-5:-3),i) = h1(:,i);
        end
    
        for i = 1:a
            phi1(6*i+(-2:0),6*i+(-5:-3)) = [0 L1(3,i) -L1(2,i); -L1(3,i) 0 L1(1,i); L1(2,i) -L1(1,i) 0];
        end
        for i = 3:a+1
            for j = 1:i-2
                phi1(6*i+(-8:-6),6*j+(-5:-3)) = phi1(6*i+(-8:-6),6*i+(-11:-9)) + phi1(6*i+(-14:-12),6*j+(-5:-3));
            end
        end
    
        phi_t1(4:6,6*a+(-5:-3)) = [0 L1(3,a) -L1(2,a); -L1(3,a) 0 L1(1,a); L1(2,a) -L1(1,a) 0];
        J1=phi_t1*phi1*H1;


        for i = 1:(n-a)
            H2(6*i+(-5:-3),i) = h2(:,i);
        end
    
        for i = 1:(n-a)
            phi2(6*i+(-2:0),6*i+(-5:-3)) = [0 L2(3,i) -L2(2,i); -L2(3,i) 0 L2(1,i); L2(2,i) -L2(1,i) 0];
        end
        for i = 3:(n-a)+1
            for j = 1:i-2
                phi2(6*i+(-8:-6),6*j+(-5:-3)) = phi2(6*i+(-8:-6),6*i+(-11:-9)) + phi2(6*i+(-14:-12),6*j+(-5:-3));
            end
        end
    
        phi_t2(4:6,6*(n-a)+(-5:-3)) = [0 L2(3,(n-a)) -L2(2,(n-a)); -L2(3,(n-a)) 0 L2(1,(n-a)); L2(2,(n-a)) -L2(1,(n-a)) 0];
        J2=phi_t2*phi2*H2;

        critical_point = closest_points(:, a);  % Artık en yakın nokta bu
        avoidance_vec = critical_point - obstacle_pos';  % Linkten engele doğru
        avoidance_dir = avoidance_vec / norm(avoidance_vec);  % Kaçınma yönü
        avoidance_speed = 0.000001;
        linear_velocity = avoidance_speed * avoidance_dir;

        dtheta1 = pinv(J1)*[0 0 0 linear_velocity']';

        dtheta2 = pinv(J2)*(Vt-J1*dtheta1);


        dtheta = [dtheta1 ;dtheta2];


        

    else %no colision risk
        for i = 1:n
            H(6*i+(-5:-3),i) = h(:,i);
        end
    
        for i = 1:n
            phi(6*i+(-2:0),6*i+(-5:-3)) = [0 L(3,i) -L(2,i); -L(3,i) 0 L(1,i); L(2,i) -L(1,i) 0];
        end
        for i = 3:n+1
            for j = 1:i-2
                phi(6*i+(-8:-6),6*j+(-5:-3)) = phi(6*i+(-8:-6),6*i+(-11:-9)) + phi(6*i+(-14:-12),6*j+(-5:-3));
            end
        end
    
        phi_t(4:6,6*n+(-5:-3)) = [0 L(3,n) -L(2,n); -L(3,n) 0 L(1,n); L(2,n) -L(1,n) 0];
    
        J=phi_t*phi*H;
        dtheta = pinv(J)*Vt;

    end


    % cla
    % plot3(obstacle_pos(1),obstacle_pos(2),obstacle_pos(3),'wo', 'MarkerSize', avoidance_radius, 'MarkerFaceColor', 'w');

    % Küreyi çizmek için koordinatlar üret
    % [xx, yy, zz] = sphere(20);  % daha fazla nokta = daha düzgün küre
    % radius = avoidance_radius;  % Gerçek birim cinsinden engel yarıçapı
    % 
    % % Küreyi engelin pozisyonuna taşı ve plotla
    % surf(radius*xx + obstacle_pos(1), ...
    %      radius*yy + obstacle_pos(2), ...
    %      radius*zz + obstacle_pos(3), ...
    %      'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.5);


    % p = [-.1 0 0; .1 0 0];
    % plot3(p(:,1), p(:,2), p(:,3), xcolor(1), 'LineWidth', 2);
    % p1 = o';
    % p2 = [0 0 .1];
    % p = [p1;p2];
    % plot3(p(:,1), p(:,2), p(:,3), xcolor(1), 'LineWidth', 2);
    % p1 = p2;
    % for i = 1:n
    %     p2 = p1 + L(:,i)';
    %     p = [p1;p2];
    %     plot3(p(:,1), p(:,2), p(:,3), xcolor(i), 'LineWidth', 6);
    %     if (i < n-1)
    %         plot3(p2(1), p2(2), p2(3), [xcolor(i) 'o'], 'LineWidth', 4);
    %     end
    %     p1 = p2;
    % end
    % k1 = .03;
    % k2 = .02;
    % p1 = p1 - k1*y(:,n)';
    % p2 = p2 + k1*y(:,n)';
    % p = [p1;p2];
    % plot3(p(:,1), p(:,2), p(:,3), xcolor(7), 'LineWidth', 3);
    % p3 = p1 + k2*x(:,n)';
    % p = [p1;p3];
    % plot3(p(:,1), p(:,2), p(:,3), xcolor(7), 'LineWidth', 3);
    % p3 = p2 + k2*x(:,n)';
    % p = [p2;p3];
    % plot3(p(:,1), p(:,2), p(:,3), xcolor(7), 'LineWidth', 3);
    % drawnow;


    conf(:,:,counter) = L;

    for i = 1:n
       k = h(:,i);
       khat = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
       R = eye(3) + sin(dtheta(i))*khat + (1-cos(dtheta(i)))*khat^2;
       x(:,i:n) = R*x(:,i:n);
       y(:,i:n) = R*y(:,i:n);
       z(:,i:n) = R*z(:,i:n);
    end
    
end

%% plot results
slider_time_series_robot_arm(conf,obstacle_pos,avoidance_radius)
