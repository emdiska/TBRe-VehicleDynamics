% plot_yaw_vs_lateral.m
% Yaw Acceleration vs Lateral Acceleration — envelope analysis
%
% FIXES APPLIED vs original:
%   FIX 1 — YawRate converted from deg/s to rad/s before differentiation
%            (max 85 deg/s is physically plausible; 85 rad/s would be ~4874
%             deg/s — impossible for a ground vehicle)
%   FIX 2 — IMU yaw rate bias removed using stationary-phase mean
%            (non-zero mean of ~0.42 deg/s indicates sensor offset;
%             removing it prevents DC error propagating into yaw acceleration)
%   FIX 3 — InlineAcc used as lateral acceleration channel
%            (IMU mounted incorrectly in car; InlineAcc is the true lateral axis)
%
% Author : Mahdi Kadiri
% Data   : TBRe 2025 car, car park testing

[fn, fp] = uigetfile({'*.csv;*.xlsx;*.xls','Logs (*.csv,*.xlsx,*.xls)'}, ...
                     'Select log file');
if isequal(fn, 0), return; end
outputFolder = uigetdir(pwd, 'Select output folder');
if isequal(outputFolder, 0), return; end

plot_yaw_vs_lateral(fullfile(fp, fn), outputFolder);

%% =========================================================================
function plot_yaw_vs_lateral(csvPath, outputPath)

% ── CONFIG ────────────────────────────────────────────────────────────────
MOVE_SPEED_THRESHOLD = 3.0;      % m/s  — samples below this excluded
DERIV_SMOOTH_WINDOW  = 5;        % samples — window for yaw rate smoothing
G0                   = 9.80665;  % m/s^2

% Column name candidates (tries each in order, uses first match)
TIME_CANDS    = ["Time","Timestamp","GPS.Time","t","secs"];
SPEED_CANDS   = ["speed","GPS.Speed","Speed","gps_speed","gps.speed"];
YAWRATE_CANDS = ["YawRate","IMU.YawRate","yaw_rate","GyroZ","gz"];

% FIX 3: InlineAcc is the true lateral channel due to IMU mounting error.
% LateralAcc (the channel labelled as lateral) is actually inline.
% Candidate list now prioritises InlineAcc.
LATERAL_CANDS = ["InlineAcc","InclineAcc","InclineAcceleration", ...
                 "Incline_Acc","Incline Acc","InlineAcceleration", ...
                 "AccelX","IMU.AccelX","ax","LateralAcc","LongAcc"];

% ── LOAD DATA ─────────────────────────────────────────────────────────────
T     = readtable(csvPath);
tcol  = findCol(T, TIME_CANDS);
scol  = findCol(T, SPEED_CANDS);
yrcol = findCol(T, YAWRATE_CANDS);
lacol = findCol(T, LATERAL_CANDS);

% Abort early with clear message if a channel is missing
assert(~isempty(tcol),  'ERROR: No time channel found. Check column names.');
assert(~isempty(scol),  'ERROR: No speed channel found. Check column names.');
assert(~isempty(yrcol), 'ERROR: No yaw rate channel found. Check column names.');
assert(~isempty(lacol), 'ERROR: No lateral acceleration channel found. Check column names.');

fprintf('Channels mapped:\n');
fprintf('  Time     -> %s\n', tcol);
fprintf('  Speed    -> %s\n', scol);
fprintf('  YawRate  -> %s\n', yrcol);
fprintf('  Lateral  -> %s  (InlineAcc used — IMU mounting correction)\n', lacol);

t   = toSeconds(T.(tcol));
v   = toNumeric(T.(scol));
yr  = toNumeric(T.(yrcol));
lat = toNumeric(T.(lacol));

% ── LATERAL ACCELERATION: convert to g ────────────────────────────────────
% Detect if already in g (|values| typically <= 3) or in m/s^2
if any(isfinite(lat)) && prctile(abs(lat(isfinite(lat))), 95) <= 3.5
    lat_g = lat;          % already in g
else
    lat_g = lat / G0;     % convert m/s^2 -> g
end

% ── FIX 1 + 2: YAW RATE — unit conversion and bias removal ───────────────
%
% FIX 1: YawRate is in deg/s (confirmed from data: max ~85 deg/s is
% physically plausible for low-speed car park manoeuvres; 85 rad/s
% would imply ~4874 deg/s — impossible for a ground vehicle).
% Convert to rad/s before differentiation so yaw acceleration is in rad/s^2.
yr_rads = yr * (pi / 180);

% FIX 2: Remove stationary-phase IMU bias.
% Non-zero mean during stationary samples (~0.42 deg/s) indicates a sensor
% offset. If uncorrected this propagates as a DC error into yaw acceleration.
stationary_mask = (v <= MOVE_SPEED_THRESHOLD) & isfinite(yr_rads);
n_stationary    = sum(stationary_mask);

if n_stationary > 10
    yr_bias  = mean(yr_rads(stationary_mask), 'omitnan');
    yr_rads  = yr_rads - yr_bias;
    fprintf('  YawRate bias removed: %.4f rad/s  (%.3f deg/s)  [%d stationary samples]\n', ...
            yr_bias, yr_bias * 180/pi, n_stationary);
else
    warning('Fewer than 10 stationary samples found — bias not removed.');
end

% ── YAW ACCELERATION ──────────────────────────────────────────────────────
% Smooth yaw rate then differentiate. gradient() uses central differences
% for interior points, one-sided at boundaries.
yr_smooth = movmean(yr_rads, DERIV_SMOOTH_WINDOW, 'omitnan');
yawAcc    = gradient(yr_smooth) ./ gradient(t);   % units: rad/s^2

% ── FILTER TO MOVING SAMPLES ──────────────────────────────────────────────
valid = (v > MOVE_SPEED_THRESHOLD) & isfinite(lat_g) & isfinite(yawAcc);
x = lat_g(valid);    % lateral acceleration [g]
y = yawAcc(valid);   % yaw acceleration [rad/s^2]

% ── ENVELOPE: quadrant farthest points ────────────────────────────────────
Q1 = find(x >= 0 & y >= 0);
Q2 = find(x <  0 & y >= 0);
Q3 = find(x >= 0 & y <  0);
Q4 = find(x <  0 & y <  0);

F1 = farthestPoint(x, y, Q1);
F2 = farthestPoint(x, y, Q2);
F3 = farthestPoint(x, y, Q3);
F4 = farthestPoint(x, y, Q4);

[~, iR] = max(x);  Pr = [x(iR) y(iR)];   % rightmost (max lateral g)
[~, iL] = min(x);  Pl = [x(iL) y(iL)];   % leftmost  (min lateral g)

% Build diamond vertices by intersecting lines between farthest points
[TopVtx, BottomVtx] = buildDiamond(F1, F2, F3, F4, Pl, Pr);
if any(~isfinite(TopVtx)),    TopVtx    = (F1 + F2) / 2; end
if any(~isfinite(BottomVtx)), BottomVtx = (F3 + F4) / 2; end

% Coverage: fraction of moving samples inside the envelope polygon
polyX    = [Pl(1) TopVtx(1) Pr(1) BottomVtx(1) Pl(1)];
polyY    = [Pl(2) TopVtx(2) Pr(2) BottomVtx(2) Pl(2)];
coverage = 100 * mean(inpolygon(x, y, polyX, polyY));

% ── PLOT ──────────────────────────────────────────────────────────────────
fig = figure('Visible', 'off');
hold on; grid on;

scatter(x, y, 8, 'filled', 'MarkerFaceColor', [0 0.4470 0.7410]);

% Front tyre limit segments (red): Pr->TopVtx and Pl->BottomVtx
hFR1 = plot([Pr(1) TopVtx(1)],    [Pr(2) TopVtx(2)],    'r', 'LineWidth', 2);
hFR2 = plot([Pl(1) BottomVtx(1)], [Pl(2) BottomVtx(2)], 'r', 'LineWidth', 2);

% Rear tyre limit segments (blue): Pl->TopVtx and Pr->BottomVtx
hRR1 = plot([Pl(1) TopVtx(1)],    [Pl(2) TopVtx(2)],    'b', 'LineWidth', 2);
hRR2 = plot([Pr(1) BottomVtx(1)], [Pr(2) BottomVtx(2)], 'b', 'LineWidth', 2);

% Vertex markers
plot(Pr(1), Pr(2), 'kd', 'MarkerFaceColor', 'y', 'MarkerSize', 6);
plot(Pl(1), Pl(2), 'ks', 'MarkerFaceColor', 'c', 'MarkerSize', 6);

% Axis labels and title
xlabel('Lateral Acceleration [g]');
ylabel('Yaw Acceleration [rad/s^2]');
[~, base] = fileparts(csvPath);
titleBase = strrep(base, '_', ' ');
title(sprintf('Yaw Acc vs Lateral Acc — %s', titleBase));

% Segment midpoint labels
rngX        = max(x) - min(x);
rngY        = max(y) - min(y);
labelOffset = 0.015 * hypot(rngX, rngY);

midAndLabelSegment(hFR1, 'Front tyre limit', [1 0 0], labelOffset);
midAndLabelSegment(hFR2, 'Front tyre limit', [1 0 0], labelOffset);
midAndLabelSegment(hRR1, 'Rear tyre limit',  [0 0 1], labelOffset);
midAndLabelSegment(hRR2, 'Rear tyre limit',  [0 0 1], labelOffset);

legend('off');

% ── SAVE ──────────────────────────────────────────────────────────────────
outFile = fullfile(outputPath, sprintf('%s_YMD_corrected.png', base));
set(fig, 'PaperPositionMode', 'auto');
print(fig, outFile, '-dpng', '-r150');
close(fig);

% ── CONSOLE SUMMARY ───────────────────────────────────────────────────────
fprintf('\n[OK] %s\n', base);
fprintf('  Coverage   : %.2f%%\n',        coverage);
fprintf('  Lat g      : min %.3f,  max %.3f g\n',     min(x), max(x));
fprintf('  YawAcc     : min %.2f,  max %.2f rad/s^2\n', min(y), max(y));
fprintf('  (pre-fix YawAcc would have been ~x57 larger due to deg->rad error)\n');
fprintf('  Saved      : %s\n', outFile);

end

%% =========================================================================
%  HELPER FUNCTIONS
%% =========================================================================

function col = findCol(T, candidates)
% Returns the first matching column name from candidates list (case-insensitive)
vars = string(T.Properties.VariableNames);
vl   = lower(vars);
col  = '';
for c = candidates
    i = find(vl == lower(c), 1);
    if ~isempty(i)
        col = char(vars(i));
        return;
    end
end
end

function x = toNumeric(col)
if iscell(col)
    x = str2double(col);
else
    x = double(col);
end
end

function t = toSeconds(col)
if isdatetime(col)
    t = seconds(col - col(1));
elseif isduration(col)
    t = seconds(col);
else
    try
        dt = datetime(col);
        t  = seconds(dt - dt(1));
    catch
        t = double(col);
    end
end
end

function P = farthestPoint(x, y, idx)
% Returns the point in idx that is farthest from the origin
if isempty(idx), P = [NaN NaN]; return; end
r2      = x(idx).^2 + y(idx).^2;
[~, k]  = max(r2);
P       = [x(idx(k))  y(idx(k))];
end

function [TopVtx, BottomVtx] = buildDiamond(F1, F2, F3, F4, Pl, Pr)
% Intersect lines between farthest quadrant points to find diamond vertices
TopVtx    = line_intersect(Pl, F2, Pr, F1);
BottomVtx = line_intersect(Pl, F4, Pr, F3);
end

function P = line_intersect(p1, p2, q1, q2)
u   = p2 - p1;
v   = q2 - q1;
den = u(1)*v(2) - u(2)*v(1);
if abs(den) < 1e-12, P = [NaN NaN]; return; end
w = q1 - p1;
s = (w(1)*v(2) - w(2)*v(1)) / den;
P = p1 + s*u;
end

function midAndLabelSegment(hLine, txt, rgb, d)
% Places a text label perpendicular to the midpoint of a line segment
XY = get(hLine, {'XData','YData'});
X  = XY{1};  Y = XY{2};
xm = mean(X); ym = mean(Y);
dx = X(2) - X(1);  dy = Y(2) - Y(1);
L  = hypot(dx, dy);
if L == 0, L = 1; end
nx = -dy/L;  ny = dx/L;           % unit perpendicular
xo = xm + d*nx;  yo = ym + d*ny;  % offset point
text(xo, yo, [' ' txt ' '], ...
     'Color', rgb, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
