% Robot kolu 3B zaman serisi görselleştirme - zaman sliderı ile
function slider_time_series_robot_arm(L_all, obstacle_pos, avoidance_radius)
    % L_all: 3x7xt array
    % obstacle_pos: 1x3

    [~, ~, T] = size(L_all);
    n = 7;
    o = zeros(3,1);
    xcolor=['b' 'r' 'r' 'g' 'g' 'w' 'w' 'g'];

    f = figure('Name','Zamana Bağlı Robot Kolu');
    axis_max = 1.0;
    axis([-axis_max axis_max -axis_max axis_max -axis_max axis_max]);
    axis off
    set(gcf, 'color', [0 0 0]);
    hold on

    % Sabit küreyi çiz
    [xx, yy, zz] = sphere(20);
    radius = avoidance_radius;
    surf(radius*xx + obstacle_pos(1), ...
         radius*yy + obstacle_pos(2), ...
         radius*zz + obstacle_pos(3), ...
         'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % Global kontrol değişkenleri
    isPlaying = false;
    currentFrame = 1;
    playSpeed = 0.1;  % başlangıç hızı (slow)

    % Slider (her hareketinde anında günceller)
    sld = uicontrol('Style','slider','Min',1,'Max',T,'Value',1,'SliderStep',[1/(T-1) 0.01], ...
        'Units','normalized','Position',[0.25 0.01 0.5 0.04]);
    addlistener(sld,'ContinuousValueChange',@(src,~) updatePlot(round(src.Value)));

    % Play/Pause butonu
    playBtn = uicontrol('Style','togglebutton','String','▶ Play',...
        'Units','normalized','Position',[0.78 0.01 0.1 0.04],...
        'Callback',@(btn,~) togglePlay());

    % Speed toggle: slow / fast
    speedToggle = uicontrol('Style','togglebutton','String','Speed: Slow', ...
        'Units','normalized','Position',[0.05 0.01 0.15 0.04], ...
        'Callback',@(btn,~) toggleSpeed());

    % Zamanlayıcı oluştur
    timerObj = timer('ExecutionMode','fixedRate','Period',playSpeed,'TimerFcn',@autoPlayStep);

    updatePlot(1);  % ilk frame

    function togglePlay()
        isPlaying = get(playBtn, 'Value');
        if isPlaying
            set(playBtn, 'String', '⏸ Pause');
            start(timerObj);
        else
            set(playBtn, 'String', '▶ Play');
            stop(timerObj);
        end
    end

    function toggleSpeed()
        if get(speedToggle, 'Value') == 1
            playSpeed = 0.001;  % fast
            set(speedToggle, 'String', 'Speed: Fast');
        else
            playSpeed = 0.1;   % slow
            set(speedToggle, 'String', 'Speed: Slow');
        end
        stop(timerObj);
        delete(timerObj);
        timerObj = timer('ExecutionMode','fixedRate','Period',playSpeed,'TimerFcn',@autoPlayStep);
        if isPlaying
            start(timerObj);
        end
    end

    function autoPlayStep(~,~)
        if isPlaying && isvalid(sld)
            currentFrame = currentFrame + 1;
            if currentFrame > T
                currentFrame = 1;
            end
            set(sld, 'Value', currentFrame);
            updatePlot(currentFrame);
        end
    end

    function updatePlot(t)
        currentFrame = t;
        cla;
        % Küreyi tekrar çiz (silindiği için)
        surf(radius*xx + obstacle_pos(1), ...
             radius*yy + obstacle_pos(2), ...
             radius*zz + obstacle_pos(3), ...
             'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

        L = L_all(:,:,t);

        % Yön vektörleri (sabit)
        x = ones(3,n); x(2:3,:) = 0;
        y = ones(3,n); y([1 3],:) = 0;
        z = ones(3,n); z(1:2,:) = 0;

        % çizim
        p1 = [0 0 0.1];
        p2 = p1;
        for j = 1:n
            p2 = p2 + L(:,j)';
            p = [p1; p2];
            plot3(p(:,1), p(:,2), p(:,3), xcolor(j), 'LineWidth', 6);
            if j < n-1
                plot3(p2(1), p2(2), p2(3), [xcolor(j) 'o'], 'LineWidth', 4);
            end
            p1 = p2;
        end

        k1 = .03;
        k2 = .02;
        p1 = p1 - k1*y(:,n)';
        p2 = p2 + k1*y(:,n)';
        p = [p1;p2];
        plot3(p(:,1), p(:,2), p(:,3), xcolor(7), 'LineWidth', 3);
        p3 = p1 + k2*x(:,n)';
        plot3([p1(1) p3(1)], [p1(2) p3(2)], [p1(3) p3(3)], xcolor(7), 'LineWidth', 3);
        p3 = p2 + k2*x(:,n)';
        plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], xcolor(7), 'LineWidth', 3);

        drawnow;
    end
end
