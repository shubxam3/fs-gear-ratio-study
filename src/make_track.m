function trk = make_track(event)
%MAKE_TRACK Track geometry for each Formula Student dynamic event.
%
%   trk = make_track(event), event = 'accel' | 'skidpad' | 'autocross' | 'endurance'
%
%   trk.s         distance along the track [m]
%   trk.kappa     curvature [1/m], zero on straights
%   trk.ds        step [m]
%   trk.laps      number of laps
%   trk.standing  true for a standing start
%
%   Acceleration and skidpad follow the rulebook geometry. The rules give
%   only guidelines for autocross and endurance, so those use a synthetic
%   lap assembled from features the guidelines allow: straights, constant
%   radius turns, a hairpin and a slalom. Replace s and kappa with a
%   surveyed centreline for a specific track.

ds = 0.5;
switch lower(event)
    case 'accel'                               % 75 m straight
        s     = (0:ds:75)';
        kappa = zeros(size(s));
        trk   = pack(s, kappa, 'Acceleration', 1, true, ds);

    case 'skidpad'                             % one timed circle, R = 9.125 m
        R     = 9.125;
        s     = (0:ds:2*pi*R)';
        kappa = ones(size(s)) / R;
        trk   = pack(s, kappa, 'Skidpad', 1, false, ds);

    case 'autocross'                           % one lap of about 1 km
        trk = synth_lap(1000, ds, 'Autocross', 1, true);

    case 'endurance'                           % same lap, 22 km in total
        trk = synth_lap(1000, ds, 'Endurance', 22, false);

    otherwise
        error('make_track:event', 'Unknown event "%s".', event);
end
end


function trk = synth_lap(L, ds, name, laps, standing)
% Segments as [length, curvature]; sign of curvature alternates left/right.
seg = [ 60        0
        pi*12     1/12        % 180 degree sweeper, R = 12 m
        40        0
        pi*9     -1/9         % R = 9 m
        30        0
        pi*10/2  -1/10        % slalom, five gates at R = 10 m
        pi*10/2   1/10
        pi*10/2  -1/10
        pi*10/2   1/10
        pi*10/2  -1/10
        50        0
        pi*7      1/7         % hairpin, R = 7 m
        35        0
        pi*15    -1/15 ];     % fast corner, R = 15 m

% Repeat the feature sequence until the lap is about L long. Repeating,
% rather than stretching, keeps every straight and corner at a legal size.
reps  = max(1, round(L / sum(seg(:,1))));
kappa = [];
for rep = 1:reps
    for q = 1:size(seg, 1)
        n     = max(1, round(seg(q,1) / ds));
        kappa = [kappa; seg(q,2) * ones(n, 1)]; %#ok<AGROW>
    end
end
kappa = [kappa; kappa(end)];
s     = (0:numel(kappa)-1)' * ds;
if laps > 1
    laps = laps * L / s(end);     % keep the total distance at laps * L
end
trk   = pack(s, kappa, name, laps, standing, ds);
end


function trk = pack(s, kappa, name, laps, standing, ds)
trk.s = s;  trk.kappa = kappa;  trk.name = name;
trk.laps = laps;  trk.standing = standing;  trk.ds = ds;
end
