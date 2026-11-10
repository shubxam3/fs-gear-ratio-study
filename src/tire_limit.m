function F_max = tire_limit(Fz, tire, direction)
%TIRE_LIMIT Largest force one tyre can transmit at normal load Fz.
%
%   direction  'x' longitudinal (drive and brake), 'y' lateral (cornering)
%
%   'constant' model: F_max = mu * Fz in both directions.
%   'magic' model:    peak of the simplified Magic Formula, with friction
%                     falling linearly with load.

Fz = max(Fz, 0);
switch tire.model
    case 'constant'
        F_max = tire.mu .* Fz;
    case 'magic'
        if direction == 'x'
            F_max = mf_fx(tire.kappa_opt, Fz, tire);
        else
            F_max = Fz .* tire.Dy .* (1 - tire.a_load .* Fz);
        end
    otherwise
        error('tire_limit:model', 'Unknown tyre model "%s".', tire.model);
end
F_max = max(F_max, 0);
end
