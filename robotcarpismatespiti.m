clc;
clear;

%% Harita ve Engellerin Oluşturulması
mapSize = [50, 50]; % Harita boyutu
map = zeros(mapSize); % Harita: 0 = Boş, 1 = Engel

% Rastgele engeller ekleme
numObstacles = 200; % Engel sayısı
rng(0); % Tekrarlanabilirlik için sabit rastgelelik
for i = 1:numObstacles
    x = randi(mapSize(1));
    y = randi(mapSize(2));
    map(x, y) = 1; % Engel ekleniyor
end

%% Başlangıç ve Hedef Noktalarının Belirlenmesi
numRobots = 10; % Robot sayısı
startPoints = zeros(numRobots, 2); % Başlangıç noktaları
goalPoints = zeros(numRobots, 2); % Hedef noktaları

for i = 1:numRobots
    while true
        startPoint = [randi(mapSize(1)), randi(mapSize(2))];
        goalPoint = [randi(mapSize(1)), randi(mapSize(2))];
        if map(startPoint(1), startPoint(2)) == 0 && map(goalPoint(1), goalPoint(2)) == 0
            startPoints(i, :) = startPoint;
            goalPoints(i, :) = goalPoint;
            break;
        end
    end
end

%% Yol Planlama (A* Algoritması)
paths = cell(numRobots, 1); % Her robot için yol
for i = 1:numRobots
    paths{i} = aStarAlgorithm(startPoints(i, :), goalPoints(i, :), map);
end

%% Hız Profili Hesaplama
robotRadii = 2; % Robot yarıçapı
maxLinearSpeed = 2; % Maksimum doğrusal hız (birim/s)
maxAngularSpeed = pi/4; % Maksimum açısal hız (radyan/s)

% Hız profili hesaplama
velocities = cell(numRobots, 1);
for i = 1:numRobots
    path = paths{i};
    if ~isempty(path)
        numPoints = size(path, 1);
        linearSpeeds = zeros(numPoints, 1);
        angularSpeeds = zeros(numPoints, 1);

        for j = 2:numPoints
            % Doğrusal hız
            distance = norm(path(j, :) - path(j-1, :));
            linearSpeeds(j) = min(maxLinearSpeed, distance);

            % Açısal hız
            if j > 2
                deltaTheta = atan2(path(j, 2) - path(j-1, 2), path(j, 1) - path(j-1, 1)) - ...
                             atan2(path(j-1, 2) - path(j-2, 2), path(j-1, 1) - path(j-2, 1));
                angularSpeeds(j) = min(maxAngularSpeed, abs(deltaTheta));
            end
        end
        velocities{i} = [linearSpeeds, angularSpeeds];
    end
end

%% Simülasyon ve Görselleştirme
figure;
imagesc(map);
colormap(gray);
hold on;
axis equal;

% Renk paleti
colors = lines(numRobots);

% Başlangıç ve hedef noktalarının işaretlenmesi
for i = 1:numRobots
    plot(startPoints(i, 2), startPoints(i, 1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Başlangıç
    plot(goalPoints(i, 2), goalPoints(i, 1), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Hedef

    % Yolların çizilmesi
    path = paths{i};
    if ~isempty(path)
        plot(path(:, 2), path(:, 1), '-', 'LineWidth', 1.5, 'Color', colors(i, :));
    end
end

% Robotların hareket animasyonu
robotMarkers = gobjects(numRobots, 1);
for i = 1:numRobots
    robotMarkers(i) = plot(startPoints(i, 2), startPoints(i, 1), '.', 'MarkerSize', 20, 'Color', colors(i, :));
end

% Simülasyon
for t = 1:100
    for i = 1:numRobots
        if t <= size(paths{i}, 1)
            currentPos = paths{i}(t, :);

            % Çarpışma Tespiti
            for j = 1:numRobots
                if i ~= j && t <= size(paths{j}, 1)
                    dist = norm(currentPos - paths{j}(t, :));
                    if dist < 2 * robotRadii
                        disp(['Çarpışma Tespiti: Robot ', num2str(i), ' ve Robot ', num2str(j)]);
                    end
                end
            end

            % Hızları yazdır
            if t <= size(velocities{i}, 1)
                linearSpeed = velocities{i}(t, 1);
                angularSpeed = velocities{i}(t, 2);
                disp(['Robot ', num2str(i), ' - Zaman Adımı: ', num2str(t), ...
                      ' - Doğrusal Hız: ', num2str(linearSpeed), ...
                      ' - Açısal Hız: ', num2str(angularSpeed)]);
            end

            % Robot pozisyonunu güncelle
            set(robotMarkers(i), 'XData', currentPos(2), 'YData', currentPos(1));
        end
    end
    pause(0.1);
end

%% Yardımcı Fonksiyonlar

% A* Algoritması
function path = aStarAlgorithm(startPoint, goalPoint, map)
    mapSize = size(map);
    openList = startPoint;
    closedList = [];
    cameFrom = zeros(mapSize(1), mapSize(2), 2);

    gCost = inf(mapSize);
    gCost(startPoint(1), startPoint(2)) = 0;

    fCost = inf(mapSize);
    fCost(startPoint(1), startPoint(2)) = heuristic(startPoint, goalPoint);

    pathFound = false;

    while ~isempty(openList)
        [~, idx] = min(fCost(sub2ind(size(fCost), openList(:, 1), openList(:, 2))));
        current = openList(idx, :);

        if isequal(current, goalPoint)
            pathFound = true;
            break;
        end

        openList(idx, :) = [];
        closedList = [closedList; current];

        neighbors = getNeighbors(current, mapSize);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);

            if map(neighbor(1), neighbor(2)) == 1 || ismember(neighbor, closedList, 'rows')
                continue;
            end

            tentativeGCost = gCost(current(1), current(2)) + 1;

            if ~ismember(neighbor, openList, 'rows')
                openList = [openList; neighbor];
            elseif tentativeGCost >= gCost(neighbor(1), neighbor(2))
                continue;
            end

            cameFrom(neighbor(1), neighbor(2), :) = current;
            gCost(neighbor(1), neighbor(2)) = tentativeGCost;
            fCost(neighbor(1), neighbor(2)) = tentativeGCost + heuristic(neighbor, goalPoint);
        end
    end

    if pathFound
        path = goalPoint;
        while ~isequal(path(1, :), startPoint)
            current = path(1, :);
            prev = squeeze(cameFrom(current(1), current(2), :))';
            path = [prev; path];
        end
    else
        path = [];
    end
end

function h = heuristic(point, goal)
    h = abs(point(1) - goal(1)) + abs(point(2) - goal(2));
end

function neighbors = getNeighbors(point, mapSize)
    neighbors = [
        point(1) - 1, point(2);
        point(1) + 1, point(2);
        point(1), point(2) - 1;
        point(1), point(2) + 1
    ];
    neighbors = neighbors(neighbors(:, 1) > 0 & neighbors(:, 1) <= mapSize(1) & ...
                          neighbors(:, 2) > 0 & neighbors(:, 2) <= mapSize(2), :);
end
