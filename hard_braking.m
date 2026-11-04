% Standalone two-car simulation: linear spring and viscous resistance.
clear; clc; close all;

% Illustrative parameters: replace with your measured values.
m1 = 1500; m2 = 1000;       % kg
b1 = 50; b2 = 40;       % N*s/m; resistance relative to ground
k = 1e6;                    % N/m, coupling spring
c = 44068;                  % N*s/m, coupling damper (modal damping ratio ~0.90)
switchTime = 1.0;            % s
scenario = 'Hard braking';
braking = true;
v0 = 20; % m/s initial forward speed of BOTH cars
% Initial spring tension balances the trailer resistance.
y0 = [b2*v0/k; v0; 0; v0];
Fbefore = (b1+b2)*v0; % N, maintains steady speed before braking
Fafter = -13200;       % N, net applied brake force on car 1
endTime = 6;

% State order: [x1; v1; x2; v2]. Displacements are referenced to
% an unstretched spring, not to a common absolute position.
coupling = @(y) k*(y(1)-y(3)) + c*(y(2)-y(4));
rhs = @(t,y,F) [y(2); ...
    (F-b1*y(2)-coupling(y))/m1; ...
    y(4); (coupling(y)-b2*y(4))/m2];
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

graphFigure = figure('Name',[scenario ' - graphs'],'Color','w');
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



% Save the plots and all solver samples alongside this script.
[folder,name] = fileparts(mfilename('fullpath'));
out = fullfile(folder,'results');
if ~isfolder(out), mkdir(out); end
exportgraphics(graphFigure,fullfile(out,[name '_graphs.pdf']),'ContentType','vector');
springForce = k*extension;
damperForce = c*(v1-v2);
data = table(t,x1,x2,v1,v2,extension,force,springForce,damperForce,springForce+damperForce, ...
    'VariableNames',{'Time_s','X1_m','X2_m','V1_m_s','V2_m_s','Extension_m', ...
                     'Applied_N','Spring_N','Damper_N','Coupling_N'});
writetable(data,fullfile(out,[name '_values.csv']));
valuesPDF = fullfile(out,[name '_values.pdf']);
writeValuesPDF(valuesPDF,data);
fprintf('Results saved in %s\n',out);

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
        'UData',0.15*sign(appliedForce),'VData',0);
    title(ax,sprintf('%s | t = %.2f s | F = %.2f N\nv1 = %.2f, v2 = %.2f m/s | one-tenth-speed playback', ...
        phase,frames(j),appliedForce,yFrames(j,2),yFrames(j,4)));
    drawnow;
end
if braking && isgraphics(fig) && t(end) < endTime
    title(ax,sprintf('Front car has stopped at t = %.3f s\nTrailer velocity = %.5f m/s; simulation ends here', ...
        t(end),v2(end)));
end

% Braking stops at the first zero crossing of the front car velocity.
% This does not model a locked brake, collision, or subsequent motion.
function [value,isterminal,direction] = frontCarStops(~,y)
    value = y(2);
    isterminal = 1;
    direction = -1;
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

