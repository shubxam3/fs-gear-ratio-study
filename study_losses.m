%% Gear ratio against time, energy and losses
%  Sweeps the gear ratio for acceleration, autocross and endurance and
%  shows what each ratio costs in time, battery energy and the individual
%  losses. No competition scoring here, only physics.

clear; clc;
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));

p       = params_generic();
events  = {'accel', 'autocross', 'endurance'};
i_sweep = 6 : 0.25 : 16;

nE = numel(events);  nI = numel(i_sweep);
T  = NaN(nI, nE);                 % time [s]
E  = NaN(nI, nE);                 % battery energy [J]
LB = NaN(nI, 4, nE);              % mean loss power [core copper gearbox inverter]

fprintf('\nGear ratio study: time, energy, losses\n');
for e = 1:nE
    trk = make_track(events{e});
    for k = 1:nI
        R = lap_sim(i_sweep(k), trk, p);
        T(k,e) = R.t_total;
        E(k,e) = R.E_batt;
        d = R.detail;
        LB(k,:,e) = [mean(d.P_core), mean(d.P_cu), mean(d.P_gb), mean(d.P_inv)];
    end
    [tb, kt] = min(T(:,e));
    [eb, ke] = min(E(:,e));
    fprintf('  %-12s fastest i = %5.2f (%8.3f s)   lowest energy i = %5.2f (%7.1f kJ)\n', ...
            trk.name, i_sweep(kt), tb, i_sweep(ke), eb/1e3);
end

%% Plots
fig = figure('Name', 'Gear ratio study: time, energy, losses', 'Position', [60 60 1300 760]);

subplot(2,2,1); hold on; grid on;
for e = 1:nE
    plot(i_sweep, T(:,e) / min(T(:,e)), 'LineWidth', 2);
end
xlabel('gear ratio i [-]'); ylabel('time relative to best [-]');
title('Time against gear ratio'); legend(events, 'Location', 'north');

subplot(2,2,2); hold on; grid on;
for e = 1:nE
    plot(i_sweep, E(:,e) / min(E(:,e)), 'LineWidth', 2);
end
xlabel('gear ratio i [-]'); ylabel('energy relative to lowest [-]');
title('Battery energy against gear ratio'); legend(events, 'Location', 'north');

subplot(2,2,3);
area(i_sweep, LB(:,:,3)); grid on;
xlabel('gear ratio i [-]'); ylabel('mean loss power [W]');
title('Loss breakdown, endurance');
legend({'core', 'copper', 'gearbox', 'inverter'}, 'Location', 'northwest');

subplot(2,2,4); hold on; grid on;
plot(i_sweep, LB(:,1,3), 'r-', 'LineWidth', 2);
plot(i_sweep, LB(:,2,3), 'b-', 'LineWidth', 2);
xlabel('gear ratio i [-]'); ylabel('mean loss power [W]');
title('Core loss rises, copper loss falls with ratio');
legend({'core', 'copper'}, 'Location', 'north');

outDir = fullfile(root, 'docs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
print(fig, fullfile(outDir, 'losses_result.png'), '-dpng', '-r150');
fprintf('Figure saved to docs/losses_result.png\n');
