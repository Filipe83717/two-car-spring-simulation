

clear;
clc;
close all;

%% Parameters — replace with your measured values
m1 = 1500;       % Mass of car 1 [kg]
m2 = 1000;       % Mass of car 2 [kg]

b1 = 50;       % Resistance coefficient for car 1 [N*s/m]
b2 = 40;       % Resistance coefficient for car 2 [N*s/m]

k = 1e6;         % Spring stiffness [N/m]

c  = 44068;     % Koblingsdemper [N*s/m], ingen koblingsdemping ved c = 0
%% Applied for  e
F0 = 1800;       % Constant force on car 1 [N]
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
    (F(t) - b1*y(2) - (k*(y(1)-y(3)) + c*(y(2)-y(4)))) / m1;
    y(4);
    ((k*(y(1)-y(3)) + c*(y(2)-y(4))) - b2*y(4)) / m2
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
%% Simulated displacements at t = 5 s
[~, idx5] = min(abs(t - 5));

fprintf('\nTime:    %.2f s\n', t(idx5));
fprintf('Car:     %.9f m\n', x1(idx5));
fprintf('Trailer: %.9f m\n', x2(idx5));

extension = x1 - x2;
spring_force = k * extension;



%% Plot results
graphFigure = figure('Color', 'w', 'Position', [100 100 850 800]);
layout = tiledlayout(graphFigure, 4, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

% Applied driving force: step from 0 to F0 at t = 0
nexttile(layout);
plot([0 0 t(end)], [0 F0 F0], 'k-', 'LineWidth', 1.5);
ylabel('Driving force [N]');
ylim([0 1.15*F0]);
xlim([0 t(end)]);
grid on;

% Displacements
nexttile(layout);
plot(t, x1, 'b-', t, x2, 'r--', 'LineWidth', 1.5);
ylabel('Displacement [m]');
legend('Car', 'Trailer', 'Location', 'best');
xlim([0 t(end)]);
grid on;

% Velocities
nexttile(layout);
plot(t, v1, 'b-', t, v2, 'r--', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');
legend('Car', 'Trailer', 'Location', 'best');
xlim([0 t(end)]);
grid on;

% Spring force
nexttile(layout);
plot(t, spring_force, 'k-', 'LineWidth', 1.5);
ylabel('Spring force [N]');
xlabel('Time [s]');
xlim([0 t(end)]);
grid on;


%% Export graph PDF, numeric PDF and CSV
[folder, name] = fileparts(mfilename('fullpath'));
out = fullfile(folder, 'results');
if ~isfolder(out), mkdir(out); end
values = table(t, x1, x2, v1, v2, extension, spring_force, ...
    'VariableNames', {'Time_s','X1_m','X2_m','V1_m_s','V2_m_s','Extension_m','SpringForce_N'});
writetable(values, fullfile(out, [name '_values.csv']));
writeValuesPDF(fullfile(out, [name '_values.pdf']), values);
exportgraphics(graphFigure, fullfile(out, [name '_graphs.pdf']), 'ContentType', 'vector');

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
function writeValuesPDF(destination,data)
    [~,reportName] = fileparts(destination);
    % Plain ASCII text on landscape A4 pages. Offsets are byte positions.
    rowsPerPage = 43;
    pages = ceil(height(data)/rowsPerPage);
    objects = cell(3+2*pages,1);
    objects{1} = '<< /Type /Catalog /Pages 2 0 R >>';
    pageIds = 4:2:(3+2*pages);
    objects{2} = sprintf('<< /Type /Pages /Count %d /Kids [%s] >>', ...
        pages,sprintf('%d 0 R ',pageIds));
    objects{3} = '<< /Type /Font /Subtype /Type1 /BaseFont /Courier >>';
    for page = 1:pages
        first = 1+(page-1)*rowsPerPage;
        last = min(first+rowsPerPage-1,height(data));
        id = pageIds(page);
        objects{id} = sprintf(['<< /Type /Page /Parent 2 0 R /MediaBox [0 0 842 595] ' ...
            '/Resources << /Font << /F1 3 0 R >> >> /Contents %d 0 R >>'],id+1);
        titleLine = sprintf('%s | samples %d-%d | page %d/%d',reportName,first,last,page,pages);
        headers = sprintf('%14s',data.Properties.VariableNames{:});
        lines = {['BT /F1 9 Tf 12 TL 24 565 Td (' titleLine ') Tj'], ...
            ['T* (' headers ') Tj']};
        for row = first:last
            lines{end+1} = ['T* (' sprintf('%14.6g',data{row,:}) ') Tj'];
        end
        stream = strjoin([lines,{'ET'}],newline);
        objects{id+1} = sprintf('<< /Length %d >>\nstream\n%s\nendstream',numel(stream),stream);
    end
    tmp = [tempname(fileparts(destination)) '.pdf'];
    fid = fopen(tmp,'wb');
    assert(fid ~= -1,'Could not open temporary PDF.');
    try
        fprintf(fid,'%%PDF-1.4\n');
        offsets = zeros(numel(objects),1);
        for id = 1:numel(objects)
            offsets(id) = ftell(fid);
            fprintf(fid,'%d 0 obj\n%s\nendobj\n',id,objects{id});
        end
        startxref = ftell(fid);
        fprintf(fid,'xref\n0 %d\n0000000000 65535 f \n',numel(objects)+1);
        fprintf(fid,'%010d 00000 n \n',offsets);
        fprintf(fid,'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n', ...
            numel(objects)+1,startxref);
        fclose(fid);
        fid = -1;
        movefile(tmp,destination,'f');
    catch err
        if fid ~= -1, fclose(fid); end
        if isfile(tmp), delete(tmp); end
        rethrow(err);
    end
end

