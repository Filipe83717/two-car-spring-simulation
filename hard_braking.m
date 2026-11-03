% Standalone two-car simulation: linear spring and viscous resistance.
clear; clc; close all;

% Illustrative parameters: replace with your measured values.
m1 = 0.20; m2 = 0.20;       % kg
b1 = 0.10; b2 = 0.10;       % N*s/m; resistance relative to ground
k = 8.0;                    % N/m; spring between cars only
switchTime = 1.0;            % s
scenario = 'Hard braking';
braking = true;
v0 = 1.0; % m/s initial forward speed of BOTH cars
% Initial spring tension balances the trailer resistance.
y0 = [b2*v0/k; v0; 0; v0];
Fbefore = (b1+b2)*v0; % N, maintains steady speed before braking
Fafter = -2.00;       % N, net applied brake force on car 1
endTime = 6;

% State order: [x1; v1; x2; v2]. Displacements are referenced to
% an unstretched spring, not to a common absolute position.
rhs = @(t,y,F) [y(2); ...
    (F-b1*y(2)-k*(y(1)-y(3)))/m1; ...
    y(4); (k*(y(1)-y(3))-b2*y(4))/m2];
opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',0.01);

% Solve separately at the force jump to avoid stepping across it.
[tA,yA] = ode45(@(t,y) rhs(t,y,Fbefore), [0 switchTime], y0, opts);
optsB = opts;
if braking
    optsB = odeset(opts,'Events',@frontCarStops);
end
[tB,yB] = ode45(@(t,y) rhs(t,y,Fafter), ...
    [switchTime endTime], yA(end,:).', optsB);
t = [tA; tB(2:end)];
y = [yA; yB(2:end,:)];
force = Fbefore*ones(size(t));
force(t >= switchTime) = Fafter;
x1 = y(:,1); v1 = y(:,2); x2 = y(:,3); v2 = y(:,4);
extension = x1-x2;

figure('Name',[scenario ' - graphs'],'Color','w');
subplot(4,1,1);
plot(t,x1,'b',t,x2,'r--','LineWidth',1.4);
ylabel('x [m]'); legend('Car 1','Car 2','Location','best'); grid on;
title(scenario);
subplot(4,1,2);
plot(t,v1,'b',t,v2,'r--','LineWidth',1.4);
ylabel('v [m/s]'); grid on;
subplot(4,1,3);
plot(t,extension,'k','LineWidth',1.4);
yline(0,':'); ylabel('Extension [m]'); grid on;
subplot(4,1,4);
stairs(t,force,'LineWidth',1.4);
ylabel('Force on car 1 [N]'); xlabel('Time [s]'); grid on;

% Animate at 30 frames/s. A slower playback makes braking easier to see.
playbackSpeed = 0.1;          % 0.1 = one-tenth speed; 1 = real time
frames = (0:playbackSpeed/30:t(end)).';
frames = unique([frames; t(end)]);
yFrames = interp1(t,y,frames,'pchip');
carLength = 0.12; carHeight = 0.06; L0 = 0.20;
% L0 is the natural coupling length; car 1 is in front of car 2.
p2 = yFrames(:,3);
p1 = yFrames(:,1)+carLength+L0;
if any(L0+yFrames(:,1)-yFrames(:,3) <= 0)
    error('Cars overlap: change parameters or model contact explicitly.');
end
fig = figure('Name',[scenario ' - animation'],'Color','w');
ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on');
axis(ax,'equal'); ylim(ax,[-0.06 0.22]);
xlabel(ax,'Track position [m]');
car1 = rectangle('Parent',ax,'Position',[p1(1) 0 carLength carHeight], ...
    'FaceColor',[0.2 0.45 0.9],'Curvature',0.15);
car2 = rectangle('Parent',ax,'Position',[p2(1) 0 carLength carHeight], ...
    'FaceColor',[0.9 0.25 0.2],'Curvature',0.15);
spring = plot(ax,nan,nan,'k-','LineWidth',1.5);
track = plot(ax,nan,nan,'k-');
label1 = text(ax,0,0.10,'Car 1','HorizontalAlignment','center');
label2 = text(ax,0,0.10,'Car 2','HorizontalAlignment','center');
arrow = quiver(ax,0,0,0,0,0,'LineWidth',2,'MaxHeadSize',0.8);
zigY = carHeight/2*ones(1,25);
zigY(2:end-1) = carHeight/2+0.012*(-1).^(1:23);
clockStart = tic;
for j = 1:numel(frames)
    if ~isgraphics(fig), break; end
    pause(max(0,frames(j)/playbackSpeed-toc(clockStart)));
    if ~isgraphics(fig), break; end
    set(car1,'Position',[p1(j) 0 carLength carHeight]);
    set(car2,'Position',[p2(j) 0 carLength carHeight]);
    set(spring,'XData',linspace(p2(j)+carLength,p1(j),25),'YData',zigY);
    set(label1,'Position',[p1(j)+carLength/2 0.10 0]);
    set(label2,'Position',[p2(j)+carLength/2 0.10 0]);
    centre = (p1(j)+p2(j)+carLength)/2;
    xlim(ax,centre+[-0.55 0.55]);
    set(track,'XData',centre+[-0.55 0.55],'YData',[-0.005 -0.005]);
    appliedForce = Fbefore;
    phase = 'Before force change';
    if frames(j) >= switchTime
        appliedForce = Fafter;
        phase = scenario;
    end
    set(arrow,'XData',p1(j)+carLength/2,'YData',0.16, ...
        'UData',0.25*appliedForce,'VData',0);
    title(ax,sprintf('%s | t = %.2f s | F = %.2f N\nv1 = %.2f, v2 = %.2f m/s | one-tenth-speed playback', ...
        phase,frames(j),appliedForce,yFrames(j,2),yFrames(j,4)));
    drawnow;
end
if braking && isgraphics(fig) && t(end) < endTime
    title(ax,sprintf('Front car has stopped at t = %.3f s\nTrailer still moving at %.3f m/s; simulation ends here', ...
        t(end),v2(end)));
end

% Braking stops at the first zero crossing of the front car velocity.
% This does not model a locked brake, collision, or subsequent motion.
function [value,isterminal,direction] = frontCarStops(~,y)
    value = y(2);
    isterminal = 1;
    direction = -1;
end
