%% No applied force after release
F = @(t) 0;

%% Start at rest with the spring stretched by 5 cm
% x1 - x2 = 0.05 m
y0 = [0.05; 0; 0; 0];

clear;
clc;
close all;

%% Parameters — replace with your measured values
m1 = 0.20;       % Mass of car 1 [kg]
m2 = 0.20;       % Mass of car 2 [kg]

b1 = 0.10;       % Resistance coefficient for car 1 [N*s/m]
b2 = 0.10;       % Resistance coefficient for car 2 [N*s/m]

k = 2.0;         % Spring stiffness [N/m]

%% Applied force
F0 = 0.10;       % Constant force on car 1 [N]
F = @(t) F0;

%% Initial conditions
% Order: [x1; v1; x2; v2]
y0 = [0; 0; 0; 0];

%% Equations of motion
% y(1) = x1
% y(2) = v1
% y(3) = x2
% y(4) = v2

dydt = @(t,y) [
    y(2);
    (F(t) - b1*y(2) - k*(y(1)-y(3))) / m1;
    y(4);
    (k*(y(1)-y(3)) - b2*y(4)) / m2
];

%% Solve from 0 to 10 seconds
tspan = linspace(0, 10, 1001);

options = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
[t, y] = ode45(dydt, tspan, y0, options);

%% Extract results
x1 = y(:,1);
v1 = y(:,2);

x2 = y(:,3);
v2 = y(:,4);

extension = x1 - x2;
spring_force = k * extension;

%% Plot results
figure;

subplot(3,1,1);
plot(t, x1, 'b', t, x2, 'r--', 'LineWidth', 1.5);
ylabel('Displacement [m]');
legend('Car 1', 'Car 2', 'Location', 'best');
title('Two cars connected by a spring');
grid on;

subplot(3,1,2);
plot(t, v1, 'b', t, v2, 'r--', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');
legend('Car 1', 'Car 2', 'Location', 'best');
grid on;

subplot(3,1,3);
plot(t, spring_force, 'k', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Spring force [N]');
grid on;

%% Animate the two cars
% Uses the t, x1 and x2 values calculated above.

carLength = 0.12;     % Visual car length [m]
carHeight = 0.06;     % Visual car height [m]
L0 = 0.20;           % Unstretched spring length [m] (for drawing)

% Actual drawing positions:
% Car 2 starts behind car 1.
p2 = x2;
p1 = x1 + carLength + L0;

fig = figure('Name', 'Two-car simulation', 'Color', 'w');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

xlabel(ax, 'Position along track [m]');
ylim(ax, [-0.04, 0.18]);

% The view follows the cars.
viewWidth = 1.0;

car2 = rectangle('Parent', ax, ...
    'Position', [p2(1), 0, carLength, carHeight], ...
    'FaceColor', [0.9 0.25 0.2], ...
    'Curvature', 0.15);

car1 = rectangle('Parent', ax, ...
    'Position', [p1(1), 0, carLength, carHeight], ...
    'FaceColor', [0.2 0.45 0.9], ...
    'Curvature', 0.15);

spring = plot(ax, NaN, NaN, 'k-', 'LineWidth', 1.5);

label1 = text(ax, p1(1), 0.09, 'Car 1', ...
    'HorizontalAlignment', 'center');
label2 = text(ax, p2(1), 0.09, 'Car 2', ...
    'HorizontalAlignment', 'center');

% Zigzag shape of the spring
nPoints = 25;
springY = (carHeight/2) * ones(1, nPoints);
springY(2:end-1) = carHeight/2 ...
    + 0.012 * (-1).^(1:nPoints-2);

%% Play animation at approximately real-time speed
frameStep = max(1, round((1/30) / mean(diff(t))));
clockStart = tic;

for j = 1:frameStep:length(t)
    if ~isgraphics(fig)
        break;
    end

    % Match animation time to simulation time.
    pause(max(0, (t(j)-t(1)) - toc(clockStart)));

    if ~isgraphics(fig)
        break;
    end

    set(car1, 'Position', ...
        [p1(j), 0, carLength, carHeight]);
    set(car2, 'Position', ...
        [p2(j), 0, carLength, carHeight]);

    % Spring connects the front of car 2 to the back of car 1.
    springX = linspace(p2(j)+carLength, p1(j), nPoints);
    set(spring, 'XData', springX, 'YData', springY);

    set(label1, 'Position', [p1(j)+carLength/2, 0.09, 0]);
    set(label2, 'Position', [p2(j)+carLength/2, 0.09, 0]);

    midpoint = (p1(j)+p2(j)+carLength)/2;
    xlim(ax, midpoint + [-viewWidth/2, viewWidth/2]);

    title(ax, sprintf('Time: %.2f s | Spring extension: %.3f m', ...
        t(j), x1(j)-x2(j)));

    drawnow;
end