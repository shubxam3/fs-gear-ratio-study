%% sweep_simulink.m
%  Gear ratio sweep through the Simulink baseline model.
%  Run build_model.m once first; it creates accel_baseline.slx.

clc;
mdl = 'accel_baseline';
cd(fileparts(mfilename('fullpath')));
load_system(mdl);

eval(model_params());            % parameters into this workspace for the plots

i_sweep = 6 : 0.05 : 16;
t_fin   = NaN(size(i_sweep));

fprintf('Running %d Simulink simulations...\n', numel(i_sweep));
for k = 1:numel(i_sweep)
    i_gear   = i_sweep(k);       %#ok<NASGU> read by the model InitFcn
    out      = sim(mdl);
    t_fin(k) = out.tout(end);    % the model stops itself at 75 m
end

[t_opt, idx] = min(t_fin);
i_opt  = i_sweep(idx);
inBand = t_fin <= t_opt + 0.01;
i_lo   = min(i_sweep(inBand));
i_hi   = max(i_sweep(inBand));

% One more run at the optimum with logged signals
i_gear = i_opt;
out    = sim(mdl);
t  = out.v_log.Time;
v  = out.v_log.Data;
a  = out.a_log.Data;
Fd = out.Fdrive_log.Data;
Ft = 2*tire.mu*(out.Fzf_log.Data + out.Fzr_log.Data);

fprintf('Optimal i = %.2f -> t_75m = %.3f s, v_end = %.1f km/h\n', i_opt, t_opt, v(end)*3.6);
fprintf('Ratios within 10 ms of the optimum: %.2f to %.2f\n', i_lo, i_hi);

figure('Name', 'Simulink baseline: gear ratio sweep', 'Position', [80 60 1200 400]);

subplot(1,3,1);
plot(i_sweep, t_fin, 'b-', 'LineWidth', 2); hold on;
xline(i_opt, '-g', 'LineWidth', 1.5, 'Label', sprintf('optimal %.2f', i_opt));
grid on; xlabel('gear ratio i [-]'); ylabel('t_{75 m} [s]'); title('Time over 75 m');

subplot(1,3,2);
plot(t, v*3.6, 'LineWidth', 1.8); hold on;
yyaxis right; plot(t, a/g, 'LineWidth', 1.2); ylabel('a [g]');
yyaxis left;  ylabel('v [km/h]');
grid on; xlabel('t [s]'); title('Speed and acceleration');

subplot(1,3,3);
plot(t, Ft, 'r-', 'LineWidth', 1.5); hold on;
plot(t, Fd, 'g-', 'LineWidth', 2);
grid on; xlabel('t [s]'); ylabel('force [N]'); title('Traction limit and drive force');
legend('traction limit', 'drive force', 'Location', 'best');
