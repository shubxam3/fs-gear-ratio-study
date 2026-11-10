function Fx = mf_fx(kappa, Fz, tire)
%MF_FX Longitudinal tyre force from a simplified Magic Formula.
%
%   Fx = Fz * mu(Fz) * sin(C * atan(B*k - E*(B*k - atan(B*k))))
%   mu(Fz) = D * (1 - a_load * Fz)      (friction falls with load)
%
%   kappa  slip ratio [-]
%   Fz     normal load on one tyre [N]
%   Fx     longitudinal force of one tyre [N]

Bk = tire.B .* kappa;
mu = tire.D .* (1 - tire.a_load .* Fz);
Fx = Fz .* mu .* sin(tire.C .* atan(Bk - tire.E .* (Bk - atan(Bk))));
end
