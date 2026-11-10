function pts = fs_points(event, value, best)
%FS_POINTS Formula Student 2026 scoring for the dynamic events (EV class).
%
%   pts = fs_points(event, value, best)
%
%   event  'Acceleration' | 'Skidpad' | 'Autocross' | 'Endurance' | 'Efficiency'
%   value  the team's time [s], or efficiency factor for 'Efficiency'
%   best   the best time (or lowest efficiency factor) in the field
%
%   Timed events:
%       T_max  = f * T_best
%       points = (P_max - P_min) * ((T_max - T) / (T_max - T_best))^2 + P_min
%
%   Efficiency (scored on the endurance run):
%       EF     = T^2 * E                 EF_max = 2 * EF_best
%       points = P_max * ((EF_max - EF) / (EF_max - EF_best))^2
%
%   Source: Formula Student Rules 2026, scoring of dynamic events.

switch event
    case 'Acceleration', P_max = 50;   f = 1.70;  P_min = 0.05 * P_max;
    case 'Skidpad',      P_max = 50;   f = 1.35;  P_min = 0.05 * P_max;
    case 'Autocross',    P_max = 100;  f = 1.40;  P_min = 0.10 * P_max;
    case 'Endurance',    P_max = 250;  f = 1.50;  P_min = 0.10 * P_max;
    case 'Efficiency',   P_max = 75;   f = 2.00;  P_min = 0;
    otherwise
        error('fs_points:event', 'Unknown event "%s".', event);
end

worst = f * best;
value = min(value, worst);
pts   = (P_max - P_min) .* ((worst - value) ./ (worst - best)).^2 + P_min;
end
