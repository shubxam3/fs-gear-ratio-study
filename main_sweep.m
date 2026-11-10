%% Gear ratio sweep for the Formula Student 75 m acceleration event
%  Runs the simulation for a range of gear ratios, picks the fastest one,
%  reports the band of ratios within 0.01 s of it and plots the result.
%
%  Usage: open this file in MATLAB and press Run.

clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

p = params_generic();

i_sweep = 6 : 0.05 : 16;      % gear ratios to test
i_ref   = 12.0;               % reference ratio to compare against
band    = 0.01;               % "as good as optimal" tolerance [s]

%% Sweep
t_fin = NaN(size(i_sweep));
v_fin = NaN(size(i_sweep));
n_fin = NaN(size(i_sweep));
for k = 1:numel(i_sweep)
    r = run_accel(i_sweep(k), p);
    if r.valid
        t_fin(k) = r.t_finish;
        v_fin(k) = r.v_finish;
        n_fin(k) = r.n_finish;
    end
end

[t_opt, idx] = min(t_fin);
i_opt  = i_sweep(idx);
inBand = t_fin <= t_opt + band;
i_lo   = min(i_sweep(inBand));
i_hi   = max(i_sweep(inBand));

res_opt = run_accel(i_opt, p);
res_ref = run_accel(i_ref, p);

%% Report
fprintf('\nGear ratio sweep, %d ratios from %.2f to %.2f\n', numel(i_sweep), i_sweep(1), i_sweep(end));
fprintf('  Optimal   i = %5.2f : t = %.3f s, v_end = %.1f km/h, motor %.0f rpm\n', ...
        i_opt, t_opt, res_opt.v_finish, res_opt.n_finish);
fprintf('  Reference i = %5.2f : t = %.3f s, v_end = %.1f km/h, motor %.0f rpm\n', ...
        i_ref, res_ref.t_finish, res_ref.v_finish, res_ref.n_finish);
fprintf('  Ratios within %.0f ms of the optimum: %.2f to %.2f\n\n', band*1e3, i_lo, i_hi);

%% Plots
cRef = [0.85 0.45 0.0];
cOpt = [0.10 0.55 0.20];

fig = figure('Name', '75 m acceleration: gear ratio sweep', 'Position', [80 60 1400 800]);

subplot(2,3,1);
plot(i_sweep, t_fin, 'b-', 'LineWidth', 2); hold on;
patch([i_lo i_hi i_hi i_lo], [t_opt t_opt t_opt+band t_opt+band], cOpt, ...
      'FaceAlpha', 0.25, 'EdgeColor', 'none');
xline(i_opt, '-',  'Color', cOpt, 'LineWidth', 1.5, 'Label', sprintf('optimal %.2f', i_opt));
xline(i_ref, '--', 'Color', cRef, 'LineWidth', 1.5, 'Label', sprintf('reference %.2f', i_ref));
grid on; xlabel('gear ratio i [-]'); ylabel('t_{75 m} [s]');
title('Time over 75 m');

subplot(2,3,2);
plot(i_sweep, n_fin, 'LineWidth', 2); hold on;
yline(p.motor.n_max*60/(2*pi),  'r--', 'LineWidth', 1.2, 'Label', 'n_{max}');
yline(p.motor.n_base*60/(2*pi), '--',  'Color', cRef, 'LineWidth', 1.2, 'Label', 'n_{base}');
xline(i_opt, '-', 'Color', cOpt, 'LineWidth', 1.5);
grid on; xlabel('gear ratio i [-]'); ylabel('motor speed at finish [rpm]');
title('Motor speed at the finish line');

subplot(2,3,3);
plot(res_opt.t, res_opt.v*3.6, '-',  'Color', cOpt, 'LineWidth', 2); hold on;
plot(res_ref.t, res_ref.v*3.6, '--', 'Color', cRef, 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('v [km/h]'); title('Speed');
legend(sprintf('optimal i = %.2f', i_opt), sprintf('reference i = %.2f', i_ref), 'Location', 'southeast');

subplot(2,3,4);
plot(res_opt.t, res_opt.a/p.g, '-',  'Color', cOpt, 'LineWidth', 2); hold on;
plot(res_ref.t, res_ref.a/p.g, '--', 'Color', cRef, 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('a [g]'); title('Acceleration');

subplot(2,3,5);
plot(res_opt.t, res_opt.Fz_f, 'b-', 'LineWidth', 1.8); hold on;
plot(res_opt.t, res_opt.Fz_r, 'r-', 'LineWidth', 1.8);
grid on; xlabel('t [s]'); ylabel('axle load [N]'); title('Axle loads (optimal ratio)');
legend('front', 'rear', 'Location', 'best');

subplot(2,3,6);
plot(res_opt.t, res_opt.Fx_f, 'b-', 'LineWidth', 1.8); hold on;
plot(res_opt.t, res_opt.Fx_r, 'r-', 'LineWidth', 1.8);
yyaxis right;
plot(res_opt.t, res_opt.P_el/1e3, 'k-', 'LineWidth', 1.2);
ylabel('electrical power [kW]');
yyaxis left; ylabel('drive force per axle [N]');
grid on; xlabel('t [s]'); title('Drive force and power (optimal ratio)');
legend('front axle', 'rear axle', 'power', 'Location', 'best');

sgtitle(sprintf('75 m acceleration  |  optimal i = %.2f  (%.2f to %.2f within %.0f ms)  |  t = %.3f s', ...
        i_opt, i_lo, i_hi, band*1e3, t_opt));

%% Save figure for the README
outDir = fullfile(fileparts(mfilename('fullpath')), 'docs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
exportgraphics(fig, fullfile(outDir, 'sweep_result.png'), 'Resolution', 150);
fprintf('Figure saved to docs/sweep_result.png\n');
