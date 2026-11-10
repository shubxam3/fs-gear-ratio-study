function str = model_params()
%MODEL_PARAMS Parameter code for the Simulink baseline model, as text.
%
%   The text is evaluated in the base workspace and stored as the model
%   InitFcn, so the model loads its own parameters on every run. i_gear is
%   only set if it does not exist yet, which lets a sweep script change it.
%
%   Representative Formula Student values, not a specific team's car.

str = strjoin({
 'motor.T_peak  = 29.1;'
 'motor.P_peak  = 35366;'
 'motor.n_base  = 11600*2*pi/60;'
 'motor.n_max   = 20000*2*pi/60;'
 'motor.J_rotor = 0.33e-3;'
 'mt = @(w) (w<=motor.n_base).*motor.T_peak + (w>motor.n_base & w<=motor.n_max).*(motor.P_peak./max(w,1e-6));'
 'motor.omega_vec = linspace(0, motor.n_max*1.1, 200);'
 'motor.T_vec     = arrayfun(mt, motor.omega_vec);'
 'veh.m=250; veh.r_tire=0.228; veh.h_cg=0.30; veh.wheelbase=1.55; veh.J_wheel=0.30;'
 'aero.rho=1.18; aero.CdA=1.30; aero.ClA=3.00; aero.balance_front=0.50;'
 'tire.mu=1.4;'
 'g=9.81;'
 'if ~exist(''i_gear'',''var'') || isempty(i_gear), i_gear=10.0; end'
 'm_eff = veh.m + (4*veh.J_wheel + 4*motor.J_rotor*i_gear^2)/veh.r_tire^2;'
 }, newline);
end
