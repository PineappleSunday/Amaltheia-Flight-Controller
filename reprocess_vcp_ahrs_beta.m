function results = reprocess_vcp_ahrs_beta(csvPath, betaValues, opts)
% REPROCESS_VCP_AHRS_BETA Replay AHRS from VCP CSV with tunable Madgwick beta.
%
% Usage:
%   results = reprocess_vcp_ahrs_beta()
%   results = reprocess_vcp_ahrs_beta("flight_log_VCP/flight_vcp_log_*.csv")
%   results = reprocess_vcp_ahrs_beta(csvPath, [0.02 0.04 0.06 0.08 0.12])
%   results = reprocess_vcp_ahrs_beta(csvPath, betaValues, opts)
%
% Inputs:
%   csvPath    Path to VCP CSV exported by flight_vcp_gui.py
%   betaValues Vector of beta values to sweep
%   opts       Optional struct:
%              - use_mag_correction (default true)
%              - make_plots (default true)
%              - roll_bias_dps (default 0)
%              - pitch_bias_dps (default 0)
%              - yaw_bias_dps (default 0)
%              - dt_min_s (default 0.001)
%              - dt_max_s (default 0.010)
%              - use_current_axis_mapping (default true)
%
% Notes:
%   - This replays the AHRS math offline using logged accel/gyro/mag.
%   - If use_current_axis_mapping=true, it matches current AHRS.c mapping:
%       roll_deg  <- atan2(-ax_pred, sqrt(ay_pred^2 + az_pred^2))
%       pitch_deg <- atan2( ay_pred, az_pred)
%       roll_rate <- gy_dps, pitch_rate <- gx_dps
%

if nargin < 1 || strlength(string(csvPath)) == 0
    csvPath = find_default_csv();
end
if nargin < 2 || isempty(betaValues)
    betaValues = [0.02 0.04 0.06 0.08 0.10 0.12 0.13 0.20 0.3 0.4 0.5 0.6 0.7 0.8 1.0] ;
end
if nargin < 3
    opts = struct();
end

opts = with_default(opts, "use_mag_correction", true);
opts = with_default(opts, "make_plots", true);
opts = with_default(opts, "roll_bias_dps", 0.0);
opts = with_default(opts, "pitch_bias_dps", 0.0);
opts = with_default(opts, "yaw_bias_dps", 0.0);
opts = with_default(opts, "dt_min_s", 0.001);
opts = with_default(opts, "dt_max_s", 0.010);
opts = with_default(opts, "use_current_axis_mapping", true);

fprintf("Loading CSV: %s\n", string(csvPath));
T = readtable(csvPath, "PreserveVariableNames", true);
N = height(T);
if N < 2
    error("CSV has insufficient rows (%d).", N);
end

[t_s, dt_s] = resolve_time_and_dt(T, N);

% Logged AHRS outputs (for overlay/error only).
logged.roll = get_numeric_column_or_nan(T, {"roll"}, N);
logged.pitch = get_numeric_column_or_nan(T, {"pitch"}, N);
logged.yaw = get_numeric_column_or_nan(T, {"yaw"}, N);
logged.roll_rate = get_numeric_column_or_nan(T, {"roll_rate"}, N);
logged.pitch_rate = get_numeric_column_or_nan(T, {"pitch_rate"}, N);
logged.yaw_rate = get_numeric_column_or_nan(T, {"yaw_rate"}, N);

% Raw sensor channels from VCP.
raw.ax = get_numeric_column_or_nan(T, {"accel_x"}, N);
raw.ay = get_numeric_column_or_nan(T, {"accel_y"}, N);
raw.az = get_numeric_column_or_nan(T, {"accel_z"}, N);
raw.mx = get_numeric_column_or_nan(T, {"mag_x"}, N);
raw.my = get_numeric_column_or_nan(T, {"mag_y"}, N);
raw.mz = get_numeric_column_or_nan(T, {"mag_z"}, N);
raw.gx = get_numeric_column_or_nan(T, {"gyro_x"}, N);
raw.gy = get_numeric_column_or_nan(T, {"gyro_y"}, N);
raw.gz = get_numeric_column_or_nan(T, {"gyro_z"}, N);

hasMag = any(isfinite(raw.mx) & isfinite(raw.my) & isfinite(raw.mz));

betaValues = betaValues(:)';
runs = cell(1, numel(betaValues));

fprintf("Replaying %d samples across %d beta values...\n", N, numel(betaValues));
for k = 1:numel(betaValues)
    beta = betaValues(k);
    run = replay_single(beta, dt_s, raw, hasMag, opts);
    run.beta = beta;

    run.roll_rmse_vs_logged = rmse_deg(run.roll_deg, logged.roll);
    run.pitch_rmse_vs_logged = rmse_deg(run.pitch_deg, logged.pitch);
    run.yaw_rmse_vs_logged = rmse_deg(wrap_deg_vec(run.yaw_deg - logged.yaw), zeros(N, 1));

    run.roll_rate_rmse_vs_logged = rmse_deg(run.roll_rate_dps, logged.roll_rate);
    run.pitch_rate_rmse_vs_logged = rmse_deg(run.pitch_rate_dps, logged.pitch_rate);
    run.yaw_rate_rmse_vs_logged = rmse_deg(run.yaw_rate_dps, logged.yaw_rate);

    runs{k} = run;
end

runs = [runs{:}];

accelCosts = arrayfun(@(r) r.accel_fit_rms, runs);
[~, bestIdx] = min(accelCosts);
best = runs(bestIdx);

summary = table(betaValues(:), ...
    arrayfun(@(r) r.accel_fit_rms, runs(:)), ...
    arrayfun(@(r) r.roll_rmse_vs_logged, runs(:)), ...
    arrayfun(@(r) r.pitch_rmse_vs_logged, runs(:)), ...
    arrayfun(@(r) r.yaw_rmse_vs_logged, runs(:)), ...
    'VariableNames', {'beta', 'accel_fit_rms', 'roll_rmse_deg', 'pitch_rmse_deg', 'yaw_rmse_deg'});

disp(summary);
fprintf("Best beta by accel-fit RMS: %.4f (RMS=%.6f)\n", best.beta, best.accel_fit_rms);

if opts.make_plots
    make_plots(t_s, logged, runs, bestIdx);
end

results = struct();
results.csvPath = string(csvPath);
results.beta_values = betaValues;
results.time_s = t_s;
results.dt_s = dt_s;
results.summary = summary;
results.best_index = bestIdx;
results.best_beta = best.beta;
results.best_run = best;
results.runs = runs;
results.options = opts;

end


function run = replay_single(beta, dt_s, raw, hasMag, opts)
N = numel(dt_s);

roll_deg = nan(N, 1);
pitch_deg = nan(N, 1);
yaw_deg = nan(N, 1);
roll_rate_dps = nan(N, 1);
pitch_rate_dps = nan(N, 1);
yaw_rate_dps = nan(N, 1);
accel_fit = nan(N, 1);
accel_trust_hist = false(N, 1);
mag_trust_hist = false(N, 1);

q = [1.0; 0.0; 0.0; 0.0]; % [w x y z]
accel_trust = true;
mag_trust = true;
mag_ref = 0.0;
mag_ref_set = false;

for i = 1:N
    dt = clampf(dt_s(i), opts.dt_min_s, opts.dt_max_s);

    % Gyro mapping from sensor frame to body frame (matches firmware).
    gx_dps = -raw.gy(i) - opts.roll_bias_dps;  % body X
    gy_dps =  raw.gx(i) - opts.pitch_bias_dps; % body Y
    gz_dps =  raw.gz(i) - opts.yaw_bias_dps;   % body Z

    ax = raw.ax(i);
    ay = raw.ay(i);
    az = raw.az(i);
    mx = raw.mx(i);
    my = raw.my(i);
    mz = raw.mz(i);

    if ~isfinite(ax), ax = 0.0; end
    if ~isfinite(ay), ay = 0.0; end
    if ~isfinite(az), az = 0.0; end
    if ~isfinite(mx), mx = 0.0; end
    if ~isfinite(my), my = 0.0; end
    if ~isfinite(mz), mz = 0.0; end
    if ~isfinite(gx_dps), gx_dps = 0.0; end
    if ~isfinite(gy_dps), gy_dps = 0.0; end
    if ~isfinite(gz_dps), gz_dps = 0.0; end

    a_mag = sqrt(ax*ax + ay*ay + az*az);
    amag_err = abs(a_mag - 1.0);
    if accel_trust
        if amag_err > 0.25, accel_trust = false; end
    else
        if amag_err < 0.15, accel_trust = true; end
    end

    if a_mag > 1e-6
        axn = ax / a_mag;
        ayn = ay / a_mag;
        azn = az / a_mag;
    else
        axn = 0.0;
        ayn = 0.0;
        azn = 0.0;
    end

    % Mag trust logic mirrors AHRS.c.
    m_mag_raw = sqrt(mx*mx + my*my + mz*mz);
    if ~opts.use_mag_correction || ~hasMag
        mag_trust = false;
    else
        if m_mag_raw < 1e-6
            mag_trust = false;
        else
            if ~mag_ref_set && accel_trust
                mag_ref = m_mag_raw;
                mag_ref_set = true;
                mag_trust = true;
            elseif mag_ref_set && accel_trust
                mag_ref = 0.999 * mag_ref + 0.001 * m_mag_raw;
            end

            if mag_ref_set
                mm_err = abs(m_mag_raw - mag_ref) / (mag_ref + 1e-6);
                if mag_trust
                    if mm_err > 0.25, mag_trust = false; end
                else
                    if mm_err < 0.15, mag_trust = true; end
                end
            else
                mag_trust = false;
            end
        end
    end

    mx_u = 0.0; my_u = 0.0; mz_u = 0.0;
    if m_mag_raw > 1e-6
        mx_u = mx / m_mag_raw;
        my_u = my / m_mag_raw;
        mz_u = mz / m_mag_raw;
    end

    gx = deg2rad(gx_dps);
    gy = deg2rad(gy_dps);
    gz = deg2rad(gz_dps);

    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

    if accel_trust
        f1 = 2.0 * (q1 * q3 - q0 * q2) - axn;
        f2 = 2.0 * (q0 * q1 + q2 * q3) - ayn;
        f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - azn;

        s0 = (-2.0 * q2) * f1 + ( 2.0 * q1) * f2;
        s1 = ( 2.0 * q3) * f1 + ( 2.0 * q0) * f2 + (-4.0 * q1) * f3;
        s2 = (-2.0 * q0) * f1 + ( 2.0 * q3) * f2 + (-4.0 * q2) * f3;
        s3 = ( 2.0 * q1) * f1 + ( 2.0 * q2) * f2;

        s_norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if s_norm > 1e-9
            s0 = s0 / s_norm;
            s1 = s1 / s_norm;
            s2 = s2 / s_norm;
            s3 = s3 / s_norm;
            qDot0 = qDot0 - beta * s0;
            qDot1 = qDot1 - beta * s1;
            qDot2 = qDot2 - beta * s2;
            qDot3 = qDot3 - beta * s3;
        end
    end

    q = q + dt * [qDot0; qDot1; qDot2; qDot3];
    q = quat_normalize(q);

    if mag_trust
        q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

        sinr_cosp = 2.0 * (q0*q1 + q2*q3);
        cosr_cosp = 1.0 - 2.0 * (q1*q1 + q2*q2);
        roll = atan2(sinr_cosp, cosr_cosp);

        sinp = 2.0 * (q0*q2 - q3*q1);
        sinp = clampf(sinp, -1.0, 1.0);
        pitch = asin(sinp);

        cr = cos(roll); sr = sin(roll);
        cp = cos(pitch); sp = sin(pitch);

        mxh = mx_u*cp + mz_u*sp;
        myh = mx_u*sr*sp + my_u*cr - mz_u*sr*cp;
        yaw_mag = atan2(-myh, mxh);

        siny_cosp = 2.0*(q0*q3 + q1*q2);
        cosy_cosp = 1.0 - 2.0*(q2*q2 + q3*q3);
        yaw_est = atan2(siny_cosp, cosy_cosp);
        yaw_err = wrap_rad(yaw_mag - yaw_est);

        if abs(yaw_err) > 1.0
            mag_trust = false;
        else
            K0 = 2.5;
            yaw_rate = abs(gz);
            K = K0 / (1.0 + 2.0 * yaw_rate);
            delta = K * yaw_err * dt;
            delta = clampf(delta, -0.05, 0.05);

            half = 0.5 * delta;
            qcorr = [cos(half); 0.0; 0.0; sin(half)];
            q = quat_mul(qcorr, q);
            q = quat_normalize(q);
        end
    end

    [~, ~, ydeg] = quat_to_euler_deg(q);

    ax_pred = 2.0 * (q(2) * q(4) - q(1) * q(3));
    ay_pred = 2.0 * (q(1) * q(2) + q(3) * q(4));
    az_pred = 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3));

    if opts.use_current_axis_mapping
        rdeg = atan2d(-ax_pred, sqrt(ay_pred * ay_pred + az_pred * az_pred));
        pdeg = atan2d( ay_pred, az_pred);
        rr_dps = gy_dps;
        pr_dps = gx_dps;
    else
        rdeg = atan2d( ay_pred, az_pred);
        pdeg = atan2d(-ax_pred, sqrt(ay_pred * ay_pred + az_pred * az_pred));
        rr_dps = gx_dps;
        pr_dps = gy_dps;
    end

    roll_deg(i) = rdeg;
    pitch_deg(i) = pdeg;
    yaw_deg(i) = wrap_deg(ydeg);
    roll_rate_dps(i) = rr_dps;
    pitch_rate_dps(i) = pr_dps;
    yaw_rate_dps(i) = gz_dps;

    accel_fit(i) = sqrt((ax_pred - axn)^2 + (ay_pred - ayn)^2 + (az_pred - azn)^2);
    accel_trust_hist(i) = accel_trust;
    mag_trust_hist(i) = mag_trust;
end

run = struct();
run.roll_deg = roll_deg;
run.pitch_deg = pitch_deg;
run.yaw_deg = yaw_deg;
run.roll_rate_dps = roll_rate_dps;
run.pitch_rate_dps = pitch_rate_dps;
run.yaw_rate_dps = yaw_rate_dps;
run.accel_fit = accel_fit;
run.accel_fit_rms = sqrt(mean(accel_fit.^2, "omitnan"));
run.accel_trust_frac = mean(accel_trust_hist);
run.mag_trust_frac = mean(mag_trust_hist);

end


function make_plots(t_s, logged, runs, bestIdx)
betas = arrayfun(@(r) r.beta, runs);
accelFit = arrayfun(@(r) r.accel_fit_rms, runs);
rollRmse = arrayfun(@(r) r.roll_rmse_vs_logged, runs);
pitchRmse = arrayfun(@(r) r.pitch_rmse_vs_logged, runs);
yawRmse = arrayfun(@(r) r.yaw_rmse_vs_logged, runs);

best = runs(bestIdx);

f = figure("Name", "AHRS Beta Replay", "Color", "w", "Position", [120 80 1500 900]);
tlo = tiledlayout(3, 2, "TileSpacing", "compact", "Padding", "compact");
title(tlo, sprintf("AHRS Replay Sweep | Best beta = %.4f", best.beta));

nexttile;
plot(betas, accelFit, "o-k", "LineWidth", 1.5, "MarkerFaceColor", [0.2 0.2 0.2]); grid on;
xlabel("Beta"); ylabel("Accel Fit RMS");
title("Objective: Predicted-vs-Measured Gravity");

nexttile;
plot(betas, rollRmse, "o-r", "LineWidth", 1.2); hold on;
plot(betas, pitchRmse, "o-g", "LineWidth", 1.2);
plot(betas, yawRmse, "o-b", "LineWidth", 1.2); grid on;
xlabel("Beta"); ylabel("RMSE (deg) vs Logged");
legend("Roll", "Pitch", "Yaw", "Location", "best");
title("Replay-vs-Logged RMSE");

nexttile;
plot(t_s, logged.roll, "k-", "LineWidth", 1.0); hold on;
plot(t_s, best.roll_deg, "r-", "LineWidth", 1.0); grid on;
xlabel("Time (s)"); ylabel("Roll (deg)");
legend("Logged", sprintf("Replay beta=%.4f", best.beta), "Location", "best");
title("Roll Overlay");

nexttile;
plot(t_s, logged.pitch, "k-", "LineWidth", 1.0); hold on;
plot(t_s, best.pitch_deg, "g-", "LineWidth", 1.0); grid on;
xlabel("Time (s)"); ylabel("Pitch (deg)");
legend("Logged", sprintf("Replay beta=%.4f", best.beta), "Location", "best");
title("Pitch Overlay");

nexttile;
plot(t_s, logged.yaw, "k-", "LineWidth", 1.0); hold on;
plot(t_s, best.yaw_deg, "b-", "LineWidth", 1.0); grid on;
xlabel("Time (s)"); ylabel("Yaw (deg)");
legend("Logged", sprintf("Replay beta=%.4f", best.beta), "Location", "best");
title("Yaw Overlay");

nexttile;
plot(t_s, best.accel_fit, "m-", "LineWidth", 1.0); grid on;
xlabel("Time (s)"); ylabel("|g_{pred} - g_{meas}|");
title("Best-Beta Instantaneous Accel Fit");
end


function qn = quat_normalize(q)
n = norm(q);
if n < 1e-9
    qn = [1.0; 0.0; 0.0; 0.0];
else
    qn = q / n;
end
end


function qout = quat_mul(a, b)
qout = zeros(4, 1);
qout(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
qout(2) = a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
qout(3) = a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
qout(4) = a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1);
end


function [roll_deg, pitch_deg, yaw_deg] = quat_to_euler_deg(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
roll_deg = rad2deg(atan2(sinr_cosp, cosr_cosp));

sinp = 2.0 * (q0 * q2 - q3 * q1);
sinp = clampf(sinp, -1.0, 1.0);
pitch_deg = rad2deg(asin(sinp));

siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
yaw_deg = rad2deg(atan2(siny_cosp, cosy_cosp));
end


function x = clampf(x, lo, hi)
if x < lo
    x = lo;
elseif x > hi
    x = hi;
end
end


function a = wrap_rad(a)
a = mod(a + pi, 2.0*pi);
if a < 0
    a = a + 2.0*pi;
end
a = a - pi;
end


function y = wrap_deg(y)
y = mod(y + 180.0, 360.0);
if y < 0
    y = y + 360.0;
end
y = y - 180.0;
end


function y = wrap_deg_vec(y)
y = mod(y + 180.0, 360.0) - 180.0;
end


function e = rmse_deg(a, b)
mask = isfinite(a) & isfinite(b);
if ~any(mask)
    e = NaN;
else
    d = a(mask) - b(mask);
    e = sqrt(mean(d.^2, "omitnan"));
end
end


function [t_s, dt_s] = resolve_time_and_dt(T, N)
t_s = get_numeric_column(T, {"host_time_s"});
if isempty(t_s)
    tick_ms = get_numeric_column(T, {"tick_ms"});
    if ~isempty(tick_ms)
        t_s = (tick_ms - tick_ms(1)) / 1000.0;
    end
end
if isempty(t_s)
    t_s = (0:N-1)' * 0.002;
end
t_s = t_s(:);
t_s = t_s - t_s(1);

dt_s = get_numeric_column(T, {"dt_sec"});
if isempty(dt_s)
    dt_s = [diff(t_s); median(diff(t_s), "omitnan")];
else
    dt_s = dt_s(:);
    if numel(dt_s) < N
        dt_s(numel(dt_s)+1:N, 1) = median(dt_s, "omitnan");
    elseif numel(dt_s) > N
        dt_s = dt_s(1:N);
    end
end

fallback_dt = median(diff(t_s), "omitnan");
if ~isfinite(fallback_dt) || fallback_dt <= 0
    fallback_dt = 0.002;
end
bad = ~isfinite(dt_s) | dt_s <= 0;
dt_s(bad) = fallback_dt;
end


function x = get_numeric_column(T, names)
x = [];
vars = string(T.Properties.VariableNames);
varsLower = lower(vars);
for i = 1:numel(names)
    key = string(names{i});
    idx = find(varsLower == lower(key), 1);
    if ~isempty(idx)
        raw = T.(vars(idx));
        if isnumeric(raw)
            x = double(raw(:));
        else
            x = str2double(string(raw(:)));
        end
        return;
    end
end
end


function x = get_numeric_column_or_nan(T, names, N)
x = get_numeric_column(T, names);
if isempty(x)
    x = nan(N, 1);
    return;
end
x = x(:);
if numel(x) < N
    x(numel(x)+1:N, 1) = nan;
elseif numel(x) > N
    x = x(1:N);
end
end


function out = with_default(in, fieldName, defaultValue)
out = in;
if ~isfield(out, fieldName) || isempty(out.(fieldName))
    out.(fieldName) = defaultValue;
end
end


function csvPath = find_default_csv()
folders = {"flight_log_VCP", "flight_log", "."};
bestFile = "";
bestDnum = -inf;

for i = 1:numel(folders)
    if ~isfolder(folders{i})
        continue;
    end
    L = dir(fullfile(folders{i}, "*.csv"));
    for k = 1:numel(L)
        if L(k).datenum > bestDnum
            bestDnum = L(k).datenum;
            bestFile = fullfile(L(k).folder, L(k).name);
        end
    end
end

if strlength(bestFile) == 0
    [f, p] = uigetfile("*.csv", "Select VCP dump CSV");
    if isequal(f, 0)
        error("No CSV selected.");
    end
    csvPath = fullfile(p, f);
else
    csvPath = bestFile;
end
end
