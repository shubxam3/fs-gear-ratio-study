%% build_model.m
%  Builds the baseline 75 m acceleration model in Simulink from a script.
%  Running it creates "accel_baseline.slx" in this folder, wires every block,
%  stores the parameters in the model InitFcn, sets the solver and does one
%  test run.
%
%  Baseline model = constant tyre friction, no accumulator power limit,
%  no gearbox losses. It is the block-diagram counterpart of run_accel.m
%  with p.tire.model = 'constant' and is used to cross-check the script.
%
%  Requires Simulink.

clc;
mdl = 'accel_baseline';
cd(fileparts(mfilename('fullpath')));

%% 0. Start from an empty model
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
open_system(mdl);

%% 1. Parameters (base workspace now, InitFcn for every later run)
paramStr = model_params();
evalin('base', paramStr);
set_param(mdl, 'InitFcn', paramStr);

%% 2. Blocks
B = @(nm, lib, pos, varargin) add_block(lib, [mdl '/' nm], 'Position', pos, varargin{:});

% Speed -> motor -> drive force
B('Gain_omega',     'built-in/Gain',       [120 40 160 70],  'Gain','i_gear/veh.r_tire');
B('Lookup_motor',   'built-in/Lookup_n-D', [220 35 290 75], ...
                    'NumberOfTableDimensions','1', ...
                    'Table','motor.T_vec', 'BreakpointsForDimension1','motor.omega_vec', ...
                    'ExtrapMethod','Clip');
B('Gain_Fmotor',    'built-in/Gain',       [350 40 390 70],  'Gain','4*i_gear/veh.r_tire');
B('MinMax_drive',   'built-in/MinMax',     [820 40 860 90],  'Function','min','Inputs','2');
B('Sum_Fnet',       'built-in/Sum',        [900 40 940 90],  'IconShape','rectangular','Inputs','+-');
B('Gain_invmeff',   'built-in/Gain',       [960 40 1000 70], 'Gain','1/m_eff');
B('Integrator_v',   'built-in/Integrator', [1040 40 1080 70],'InitialCondition','0.05');
B('Integrator_x',   'built-in/Integrator', [1120 40 1160 70],'InitialCondition','0');
B('Compare_x',      'simulink/Logic and Bit Operations/Compare To Constant', ...
                    [1220 40 1280 70], 'relop','>=', 'const','75');
B('Stop',           'built-in/Stop',       [1220 120 1260 150]);

% Aero
B('Prod_vsq',       'built-in/Product',    [120 170 160 210],'Inputs','2');
B('Gain_drag',      'built-in/Gain',       [240 150 280 180],'Gain','0.5*aero.rho*aero.CdA');
B('Gain_down',      'built-in/Gain',       [240 230 280 260],'Gain','0.5*aero.rho*aero.ClA');

% Wheel loads (per wheel)
B('Gain_aerofront', 'built-in/Gain',       [360 230 400 260],'Gain','aero.balance_front/2');
B('Gain_aerorear',  'built-in/Gain',       [360 300 400 330],'Gain','(1-aero.balance_front)/2');
B('Const_Fzstatic', 'built-in/Constant',   [240 380 300 410],'Value','veh.m*g/4');
B('Memory_a',       'built-in/Memory',     [120 300 160 330],'InitialCondition','0');
B('Gain_loadtrans', 'built-in/Gain',       [240 300 280 330],'Gain','veh.m*veh.h_cg/(2*veh.wheelbase)');
B('Sum_Fzf',        'built-in/Sum',        [500 225 540 285],'IconShape','rectangular','Inputs','++-');
B('Sum_Fzr',        'built-in/Sum',        [500 320 540 380],'IconShape','rectangular','Inputs','+++');
B('Sat_Fzf',        'built-in/Saturate',   [600 240 640 270],'UpperLimit','inf','LowerLimit','0');
B('Sat_Fzr',        'built-in/Saturate',   [600 335 640 365],'UpperLimit','inf','LowerLimit','0');

% Traction limit
B('Sum_Fz',         'built-in/Sum',        [690 290 730 340],'IconShape','rectangular','Inputs','++');
B('Gain_traction',  'built-in/Gain',       [760 295 800 325],'Gain','2*tire.mu');

% Logging
ws = {'SaveFormat','Timeseries','SampleTime','-1'};
B('ToWs_v',     'built-in/ToWorkspace', [1120 150 1180 180],'VariableName','v_log',      ws{:});
B('ToWs_x',     'built-in/ToWorkspace', [1300 40  1360 70], 'VariableName','x_log',      ws{:});
B('ToWs_a',     'built-in/ToWorkspace', [1040 150 1100 180],'VariableName','a_log',      ws{:});
B('ToWs_Fdrive','built-in/ToWorkspace', [820  150 900  180],'VariableName','Fdrive_log', ws{:});
B('ToWs_Fzf',   'built-in/ToWorkspace', [690  200 750  230],'VariableName','Fzf_log',    ws{:});
B('ToWs_Fzr',   'built-in/ToWorkspace', [690  410 750  440],'VariableName','Fzr_log',    ws{:});

%% 3. Signal lines
L = @(s,d) add_line(mdl, s, d, 'autorouting','on');

L('Integrator_v/1','Gain_omega/1');
L('Integrator_v/1','Prod_vsq/1');
L('Integrator_v/1','Prod_vsq/2');
L('Integrator_v/1','Integrator_x/1');
L('Integrator_v/1','ToWs_v/1');

L('Gain_omega/1','Lookup_motor/1');
L('Lookup_motor/1','Gain_Fmotor/1');
L('Gain_Fmotor/1','MinMax_drive/1');

L('Prod_vsq/1','Gain_drag/1');
L('Prod_vsq/1','Gain_down/1');
L('Gain_drag/1','Sum_Fnet/2');
L('Gain_down/1','Gain_aerofront/1');
L('Gain_down/1','Gain_aerorear/1');

L('Const_Fzstatic/1','Sum_Fzf/1');
L('Const_Fzstatic/1','Sum_Fzr/1');
L('Gain_aerofront/1','Sum_Fzf/2');
L('Gain_aerorear/1','Sum_Fzr/2');
L('Memory_a/1','Gain_loadtrans/1');   % previous-step acceleration breaks the algebraic loop
L('Gain_loadtrans/1','Sum_Fzf/3');    % front unloads
L('Gain_loadtrans/1','Sum_Fzr/3');    % rear loads up
L('Sum_Fzf/1','Sat_Fzf/1');
L('Sum_Fzr/1','Sat_Fzr/1');
L('Sat_Fzf/1','Sum_Fz/1');
L('Sat_Fzr/1','Sum_Fz/2');
L('Sat_Fzf/1','ToWs_Fzf/1');
L('Sat_Fzr/1','ToWs_Fzr/1');

L('Sum_Fz/1','Gain_traction/1');
L('Gain_traction/1','MinMax_drive/2');

L('MinMax_drive/1','Sum_Fnet/1');
L('MinMax_drive/1','ToWs_Fdrive/1');
L('Sum_Fnet/1','Gain_invmeff/1');
L('Gain_invmeff/1','Integrator_v/1');
L('Gain_invmeff/1','Memory_a/1');
L('Gain_invmeff/1','ToWs_a/1');

L('Integrator_x/1','Compare_x/1');
L('Integrator_x/1','ToWs_x/1');
L('Compare_x/1','Stop/1');

%% 4. Solver
set_param(mdl, 'SolverType','Fixed-step', 'Solver','ode4', ...
               'FixedStep','0.001', 'StopTime','30');

%% 5. Layout (cosmetic)
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch
    % older releases: keep the manual positions
end

%% 6. Save and test run
save_system(mdl);
fprintf('Model "%s.slx" built and saved.\n', mdl);

try
    out = sim(mdl);
    fprintf('Test run OK: i_gear = %.2f -> t_75m = %.3f s\n', ...
            evalin('base','i_gear'), out.tout(end));
catch ME
    fprintf(2, 'Model built, but the test run failed: %s\n', ME.message);
end
