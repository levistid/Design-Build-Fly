%% DBF Motor / Prop / Battery Sizing Tool
% Solves motor/prop equilibrium at specified flight conditions
clear; clc;

%% ===================== USER INPUTS ===============================
% ---- Battery ----
V_nom = 22.2;         % nominal voltage of 6S (V)
V_full = 25.2;        % full charge voltage (V)
Cap_Ah = 4.4;         % battery capacity (Ah)
R_internal = 0.015;   % pack internal resistance (ohms) - typical for 80C

% ---- Motor (per motor) ----
Kv = 700;             % motor KV (RPM/V)
Rm = 0.04;            % motor winding resistance (ohms)
Io = 1.2;             % no-load current (A)
motor_max_power = 1000;  % W (continuous per motor)
motor_mass = 0.22;       % kg

% ---- Propeller ----
D = 14;               % diameter (inches)
P = 7;                % pitch (inches)

% Propeller coefficients vs advance ratio J
% Ct(J) = Ct0 + Ct1*J + Ct2*J^2 (fitted from APC data)
Ct_poly = [0.110, -0.065, 0.015];   % [Ct0, Ct1, Ct2]
Cp_poly = [0.042, 0.025, 0.008];    % [Cp0, Cp1, Cp2]

% ---- Aircraft ----
weight_N = 45;        % aircraft weight (N) (~4.6 kg)
num_motors = 2;

% ---- Flight Conditions to Analyze ----
% Format: [V_aircraft(m/s), throttle]
flight_conditions = [
    0,   1.0;   % Static Thrust
    8,   0.95;  % Takeoff Roll
    12,  0.75;  % Climb
    16,  0.55;  % Cruise
    20,  0.85;  % Sprint
];

flight_names = {'Static Thrust', 'Takeoff Roll', 'Climb', 'Cruise', 'Sprint'};

%% ===================== CONSTANTS ===============================
D_m = D * 0.0254;     % diameter in meters
rho = 1.225;          % air density (kg/m^3)
Kt = 60/(2*pi*Kv);    % motor torque constant (N*m/A)

%% ===================== ANALYSIS LOOP ===============================
fprintf('======================================================\n');
fprintf('DBF MOTOR/PROP SIZING ANALYSIS\n');
fprintf('Motor: %.0f KV, Prop: %.0fx%.0f", Battery: 6S %.1fAh\n', Kv, D, P, Cap_Ah);
fprintf('Aircraft Weight: %.1f N (%.1f kg)\n', weight_N, weight_N/9.81);
fprintf('======================================================\n\n');

results = struct();

for cond = 1:size(flight_conditions, 1)
    condition_name = flight_names{cond};
    V_aircraft = flight_conditions(cond, 1);
    throttle = flight_conditions(cond, 2);
    
    %% Iterative Solution for Motor/Prop Equilibrium
    V_applied = throttle * V_full;  % Commanded voltage
    RPM_solution = 0;
    I_motor = 0;
    converged = false;
    
    % Initial guess: no-load RPM
    RPM = Kv * V_applied * 0.8;  % Start at 80% of no-load
    
    for iter = 1:50
        n_rps = RPM / 60;  % rev/s
        
        % Advance ratio
        if n_rps * D_m < 0.01
            J = 0;  % Static condition
        else
            J = V_aircraft / (n_rps * D_m);
        end
        J = max(min(J, 1.2), 0);  % Clamp to valid range
        
        % Propeller coefficients at this J
        Ct = Ct_poly(1) + Ct_poly(2)*J + Ct_poly(3)*J^2;
        Cp = Cp_poly(1) + Cp_poly(2)*J + Cp_poly(3)*J^2;
        
        % Propeller load torque
        Q_prop = Cp * rho * n_rps^2 * D_m^5;
        
        % Required motor current to produce this torque
        % Motor equation: Kt * (I - Io) = Q_prop
        I_motor = (Q_prop / Kt) + Io;
        
        % Voltage equation: V_applied = I*Rm + RPM/Kv
        % Solve for RPM: RPM = Kv * (V_applied - I*Rm)
        RPM_new = Kv * (V_applied - I_motor * Rm);
        
        % Check convergence
        if abs(RPM_new - RPM) < 1
            RPM_solution = RPM_new;
            converged = true;
            break;
        end
        
        % Update with damping for stability
        RPM = 0.5 * RPM + 0.5 * RPM_new;
    end
    
    if ~converged
        fprintf('WARNING: Did not converge for %s\n', condition_name);
    end
    
    %% Calculate Performance Metrics
    n_rps = RPM_solution / 60;
    
    % Recalculate at final RPM
    if n_rps * D_m < 0.01
        J = 0;
    else
        J = V_aircraft / (n_rps * D_m);
    end
    J = max(min(J, 1.2), 0);
    
    Ct = Ct_poly(1) + Ct_poly(2)*J + Ct_poly(3)*J^2;
    Cp = Cp_poly(1) + Cp_poly(2)*J + Cp_poly(3)*J^2;
    
    % Thrust (per motor)
    T_single = Ct * rho * n_rps^2 * D_m^4;
    T_total = T_single * num_motors;
    
    % Power
    P_mech = Cp * rho * n_rps^3 * D_m^5;
    V_terminal = V_applied - I_motor * (Rm + R_internal/num_motors);
    P_elec = V_terminal * I_motor;
    
    % Efficiency
    if P_elec > 0
        eta_prop = (J * Ct / Cp);
        eta_motor = P_mech / P_elec;
        eta_total = eta_prop * eta_motor;
    else
        eta_prop = 0;
        eta_motor = 0;
        eta_total = 0;
    end
    
    % Total system
    I_total = I_motor * num_motors;
    P_total = P_elec * num_motors;
    TWR = T_total / weight_N;
    
    %% Store Results
    results(cond).name = condition_name;
    results(cond).V_aircraft = V_aircraft;
    results(cond).throttle = throttle;
    results(cond).RPM = RPM_solution;
    results(cond).J = J;
    results(cond).I_motor = I_motor;
    results(cond).I_total = I_total;
    results(cond).T_single = T_single;
    results(cond).T_total = T_total;
    results(cond).TWR = TWR;
    results(cond).P_elec = P_elec;
    results(cond).P_total = P_total;
    results(cond).eta_total = eta_total;
    
    %% Display
    fprintf('--- %s ---\n', condition_name);
    fprintf('  Airspeed:         %.1f m/s (%.1f mph)\n', V_aircraft, V_aircraft*2.237);
    fprintf('  Throttle:         %.0f%%\n', throttle*100);
    fprintf('  RPM:              %.0f\n', RPM_solution);
    fprintf('  Advance Ratio J:  %.3f\n', J);
    fprintf('  Current/motor:    %.1f A\n', I_motor);
    fprintf('  Total Current:    %.1f A\n', I_total);
    fprintf('  Thrust/motor:     %.2f N (%.2f lb)\n', T_single, T_single*0.2248);
    fprintf('  Total Thrust:     %.2f N (%.2f lb)\n', T_total, T_total*0.2248);
    fprintf('  Thrust-to-Weight: %.2f\n', TWR);
    fprintf('  Power/motor:      %.0f W\n', P_elec);
    fprintf('  Total Power:      %.0f W\n', P_total);
    fprintf('  Efficiency:       %.1f%%\n', eta_total*100);
    fprintf('\n');
end

%% ===================== MISSION ENDURANCE ESTIMATE ===============================
fprintf('======================================================\n');
fprintf('ENDURANCE ESTIMATE\n');
fprintf('======================================================\n');

% Simple mission profile (weighted average)
% Assume: 10% takeoff, 15% climb, 60% cruise, 15% maneuver
mission_profile = [
    results(2).I_total, 0.10;   % Takeoff
    results(3).I_total, 0.15;   % Climb
    results(4).I_total, 0.60;   % Cruise
    results(5).I_total, 0.15;   % Sprint/Maneuver
];

I_avg = sum(mission_profile(:,1) .* mission_profile(:,2));
fprintf('Average Current Draw: %.1f A\n', I_avg);

% Battery usable capacity (80% DoD rule)
Cap_usable = Cap_Ah * 0.8;
flight_time_hr = Cap_usable / I_avg;
flight_time_min = flight_time_hr * 60;

fprintf('Usable Capacity:      %.2f Ah (80%% DoD)\n', Cap_usable);
fprintf('Estimated Flight Time: %.1f min\n', flight_time_min);
fprintf('Energy Available:     %.0f Wh\n', V_nom * Cap_usable);

%% ===================== DESIGN RECOMMENDATIONS ===============================
fprintf('\n======================================================\n');
fprintf('DESIGN RECOMMENDATIONS\n');
fprintf('======================================================\n');

% Check thrust margin
TWR_cruise = results(4).TWR;
if TWR_cruise < 1.2
    fprintf('⚠ LOW THRUST MARGIN (T/W=%.2f) - Consider:\n', TWR_cruise);
    fprintf('   • Higher KV motor (more RPM)\n');
    fprintf('   • Larger prop diameter\n');
    fprintf('   • Higher throttle in cruise\n');
elseif TWR_cruise < 1.5
    fprintf('✓ Adequate thrust margin (T/W=%.2f)\n', TWR_cruise);
else
    fprintf('✓ Good thrust margin (T/W=%.2f)\n', TWR_cruise);
end

% Check static thrust
TWR_static = results(1).TWR;
if TWR_static < 1.5
    fprintf('⚠ LOW STATIC THRUST (T/W=%.2f) - Takeoff may be long\n', TWR_static);
else
    fprintf('✓ Good static thrust (T/W=%.2f)\n', TWR_static);
end

% Check efficiency
eta_cruise = results(4).eta_total;
if eta_cruise < 0.50
    fprintf('⚠ LOW EFFICIENCY (%.0f%%) - Check prop selection\n', eta_cruise*100);
elseif eta_cruise < 0.65
    fprintf('⚠ Moderate efficiency (%.0f%%) - Could improve\n', eta_cruise*100);
else
    fprintf('✓ Good efficiency (%.0f%%)\n', eta_cruise*100);
end

% Check endurance
if flight_time_min < 4
    fprintf('⚠ SHORT ENDURANCE (%.1f min) - Need larger battery\n', flight_time_min);
elseif flight_time_min < 6
    fprintf('⚠ Marginal endurance (%.1f min)\n', flight_time_min);
else
    fprintf('✓ Good endurance (%.1f min)\n', flight_time_min);
end

% Check current limits
I_max_continuous = Cap_Ah * 80;  % 80C rating
I_peak = max([results.I_total]);
if I_peak > I_max_continuous
    fprintf('⚠ EXCEEDS BATTERY C-RATING (%.0fA > %.0fA max)\n', I_peak, I_max_continuous);
else
    fprintf('✓ Within battery limits (%.0fA < %.0fA max)\n', I_peak, I_max_continuous);
end

fprintf('======================================================\n');


%% END SCRIPT
