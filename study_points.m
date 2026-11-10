%% Points-optimal gear ratio over all dynamic events
%  Runs every gear ratio through acceleration, skidpad, autocross and
%  endurance, converts times and energy to points with the Formula Student
%  2026 formulas and finds the ratio with the highest total.
%
%  There is no competitor data, so the reference for each event is the best
%  result inside the sweep. The points therefore show what an off-optimal
%  ratio loses, not a predicted competition score.

clear; clc;
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));

p       = params_generic();
i_sweep = (6 : 0.25 : 16)';
nI      = numel(i_sweep);

names  = {'Acceleration', 'Skidpad', 'Autocross', 'Endurance'};
tracks = {'accel', 'skidpad', 'autocross', 'endurance'};
t = NaN(nI, 4);  E_end = NaN(nI, 1);

for e = 1:4
    trk = make_track(tracks{e});
    for k = 1:nI
        if e == 1
            R = run_accel(i_sweep(k), p);         % time-domain model for the sprint
            t(k,e) = R.t_finish;
        else
            R = lap_sim(i_sweep(k), trk, p);
            t(k,e) = R.t_total;
            if e == 4, E_end(k) = R.E_batt; end
        end
    end
end

% Points per event
P = NaN(nI, 5);
for e = 1:4
    P(:,e) = fs_points(names{e}, t(:,e), min(t(:,e)));
end
EF     = t(:,4).^2 .* E_end;                      % efficiency factor
P(:,5) = fs_points('Efficiency', EF, min(EF));

total       = sum(P, 2);
[tot_opt, kb] = max(total);
i_opt       = i_sweep(kb);
near        = total >= tot_opt - 1;               % within one point of the best
P_max       = [50 50 100 250 75];

%% Report
labels = [names, {'Efficiency'}];
fprintf('\nPoints-optimal gear ratio: i = %.2f, %.1f of 525 points\n', i_opt, tot_opt);
fprintf('Within one point of the best: i = %.2f to %.2f\n', min(i_sweep(near)), max(i_sweep(near)));
for e = 1:5
    [~, ke] = max(P(:,e));
    fprintf('  %-12s %6.1f / %3d   (event optimum at i = %.2f)\n', ...
            labels{e}, P(kb,e), P_max(e), i_sweep(ke));
end

%% Plots
fig = figure('Name', 'Points-optimal gear ratio', 'Position', [40 60 1300 760]);

subplot(2,2,[1 3]);
imagesc(i_sweep, 1:5, (P ./ repmat(P_max, nI, 1))');
set(gca, 'YTick', 1:5, 'YTickLabel', ...
    {'Accel (50)', 'Skidpad (50)', 'Autocross (100)', 'Endurance (250)', 'Efficiency (75)'});
caxis([0.5 1]); colorbar;
hold on; plot([i_opt i_opt], [0.5 5.5], 'w--', 'LineWidth', 2);
xlabel('gear ratio i [-]'); title('Share of each event''s points');

subplot(2,2,2);
area(i_sweep, [sum(P(:,1:4), 2), P(:,5)]); grid on;
xlabel('gear ratio i [-]'); ylabel('points');
title('Time-scored events and efficiency');
legend('time-scored events', 'efficiency', 'Location', 'south');

subplot(2,2,4);
plot(i_sweep, total, 'k-', 'LineWidth', 2.5); grid on; hold on;
plot(i_opt, tot_opt, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.1 0.55 0.2], 'MarkerEdgeColor', 'k');
xlabel('gear ratio i [-]'); ylabel('total points (of 525)');
title(sprintf('Total points, optimum at i = %.2f', i_opt));

outDir = fullfile(root, 'docs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
print(fig, fullfile(outDir, 'points_result.png'), '-dpng', '-r150');
fprintf('Figure saved to docs/points_result.png\n');
