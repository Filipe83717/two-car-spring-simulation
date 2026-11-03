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