clear; clc;
%% ========================================================================
%   AIAA DBF POWER SYSTEM SIMULATION - PHYSICS-BASED MODEL
%   For: Twin-motor cargo aircraft with 6S LiPo (HCL 22.2V 4400mAh 80C)
% =========================================================================

%% ========================
%   BATTERY PARAMETERS
% =========================
V_cell_nom = 3.7;       % Nominal voltage per cell
V_cell_full = 4.2;      % Fully charged per cell
V_cell_min = 3.3;       % Cutoff voltage per cell
n_cells = 6;            % 6S configuration
V_nom = V_cell_nom * n_cells;
V_full = V_cell_full * n_cells;
V_min = V_cell_min * n_cells;

Q_batt = 4.4;           % Ah capacity
C_rating = 80;          % 80C continuous discharge
I_max = Q_batt * C_rating;  % 352A max continuous
R_internal = 0.010;     % Ohms (measured under load)

%% ========================
%   AIRCRAFT PARAMETERS
% =========================
mass = 3.5;             % kg (including payload)
S = 0.48;               % m^2 wing area
b = 1.96;               % m wingspan (for AR=8)
AR = b^2/S;             % Aspect ratio
e = 0.85;               % Oswald efficiency factor

% Airfoil: NACA 4412
CL_max = 1.6;           % Max lift coefficient (clean)
CL_cruise = 0.6;        % Typical cruise CL
CD0 = 0.035;            % Zero-lift drag coefficient
k = 1/(pi*e*AR);        % Induced drag factor (0.0468 for your AR)

% Environment
rho = 1.225;            % kg/m^3 (sea level)
g = 9.81;               % m/s^2

%% ========================
%   MOTOR PARAMETERS (x2)
% =========================
n_motors = 2;
KV = 600;               % RPM/V
KV_rad = KV * (2*pi/60);% rad/s/V
Rm = 0.055;             % Ohms motor resistance
I0 = 2.0;               % No-load current (A)

%% ========================
%   PROPELLER MODEL: 14x8
% =========================
% APC 14x8E style propeller coefficients
% Based on UIUC propeller database
prop_diam = 14 * 0.0254;  % Convert inches to meters
prop_pitch = 8 * 0.0254;  % m

% Thrust coefficient: Ct = a0 + a1*J + a2*J^2
Ct_coeff = [0.1200, -0.0850, 0.0200];  % Empirical fit
% Power coefficient: Cp = b0 + b1*J + b2*J^2
Cp_coeff = [0.0450, 0.0280, 0.0100];   % Empirical fit
% Efficiency: eta = J*Ct/Cp

%% ========================
%   MISSION PROFILE
% =========================
% [t_start  t_end  flight_phase  throttle_initial  target_velocity]
% Flight phases: 1=Takeoff, 2=Climb, 3=Cruise, 4=Maneuver, 5=Descent
mission = [
    0    10   1   0.95   20    % Takeoff roll & initial climb
    10   40   2   0.70   18    % Climb to pattern altitude
    40   160  3   0.50   16    % Cruise laps (main mission)
    160  190  4   0.75   17    % Maneuvering (pylon turns)
    190  240  3   0.45   15    % Final cruise
    240  260  5   0.25   12    % Descent & landing approach
];

%% ========================
%   SIMULATION SETUP
% =========================
dt = 0.1;               % Time step (s)
t_final = mission(end,2);
t = 0:dt:t_final;
N = length(t);

% Pre-allocate arrays
throttle = zeros(1,N);
V_aircraft = zeros(1,N);
CL = zeros(1,N);
CD = zeros(1,N);
Thrust_req = zeros(1,N);
Thrust_avail = zeros(1,N);
Drag = zeros(1,N);
Power_mech = zeros(1,N);
Power_elec = zeros(1,N);

RPM = zeros(1,N);
I_motor = zeros(1,N);
I_total = zeros(1,N);
V_batt = zeros(1,N);
SOC = zeros(1,N);
Capacity_used = zeros(1,N);

% Initial conditions
SOC(1) = 1.0;
V_batt(1) = V_full;
V_aircraft(1) = 12;     % Initial velocity (m/s)

%% ========================================================================
%                           MAIN SIMULATION LOOP
% =========================================================================
fprintf('Running DBF Mission Simulation...\n');

for i = 2:N
    %% ---- 1. Determine Flight Phase & Throttle Setting ----
    phase = 3;  % Default to cruise
    throttle_cmd = 0.5;
    V_target = 15;
    
    for seg = 1:size(mission,1)
        if t(i) >= mission(seg,1) && t(i) < mission(seg,2)
            phase = mission(seg,3);
            throttle_cmd = mission(seg,4);
            V_target = mission(seg,5);
            break;
        end
    end
    
    %% ---- 2. Flight Mechanics (Velocity & Required Thrust) ----
    % Target velocity with simple acceleration model
    V_aircraft(i) = V_aircraft(i-1) + 0.1*(V_target - V_aircraft(i-1));
    V_aircraft(i) = max(V_aircraft(i), 8);  % Min flying speed
    
    % Lift coefficient required for steady flight
    CL(i) = (2 * mass * g) / (rho * V_aircraft(i)^2 * S);
    CL(i) = min(CL(i), CL_max);  % Stall protection
    
    % Drag calculation
    CD(i) = CD0 + k * CL(i)^2;
    Drag(i) = 0.5 * rho * V_aircraft(i)^2 * S * CD(i);
    
    % Required thrust (D + acceleration term for climb)
    if phase == 1 || phase == 2  % Takeoff/Climb
        Thrust_req(i) = Drag(i) + 0.15*mass*g;  % Excess for climb
    else
        Thrust_req(i) = Drag(i);
    end
    
    %% ---- 3. Propeller Performance (Advance Ratio Method) ----
    % Iterate to find required throttle
    throttle(i) = throttle_cmd;
    
    for iter = 1:5  % Simple iteration for thrust matching
        % Motor speed from back-EMF
        omega = throttle(i) * KV_rad * V_batt(i-1);
        n_rps = omega / (2*pi);  % Revolutions per second
        RPM(i) = n_rps * 60;
        
        % Advance ratio: J = V/(n*D)
        J = V_aircraft(i) / (n_rps * prop_diam);
        J = max(min(J, 1.2), 0.1);  % Clamp to valid range
        
        % Thrust & power coefficients (per motor)
        Ct = Ct_coeff(1) + Ct_coeff(2)*J + Ct_coeff(3)*J^2;
        Cp = Cp_coeff(1) + Cp_coeff(2)*J + Cp_coeff(3)*J^2;
        
        % Thrust per motor: T = Ct * rho * n^2 * D^4
        T_single = Ct * rho * n_rps^2 * prop_diam^4;
        Thrust_avail(i) = n_motors * T_single;
        
        % Adjust throttle if needed (simple proportional control)
        if Thrust_avail(i) < Thrust_req(i) * 0.95
            throttle(i) = min(throttle(i) * 1.05, 1.0);
        elseif Thrust_avail(i) > Thrust_req(i) * 1.05
            throttle(i) = max(throttle(i) * 0.95, 0.1);
        else
            break;  % Converged
        end
    end
    
    %% ---- 4. Motor Electrical Model ----
    % Mechanical power per motor: P = Cp * rho * n^3 * D^5
    P_mech_single = Cp * rho * n_rps^3 * prop_diam^5;
    Power_mech(i) = n_motors * P_mech_single;
    
    % Motor current per motor (back-EMF equation)
    % V_batt = I*Rm + RPM/KV
    I_motor(i) = (V_batt(i-1) - RPM(i)/KV) / Rm;
    I_motor(i) = max(I_motor(i), I0);  % At least no-load current
    I_motor(i) = min(I_motor(i), I_max/(2*n_motors));  % Current limit
    
    % Total current draw
    I_total(i) = n_motors * I_motor(i);
    
    % Electrical power
    Power_elec(i) = V_batt(i-1) * I_total(i);
    
    %% ---- 5. Battery Model ----
    % Voltage sag from internal resistance
    V_batt(i) = V_full - I_total(i) * R_internal - ...
                (1-SOC(i-1)) * 0.5 * n_cells;  % Discharge curve
    V_batt(i) = max(V_batt(i), V_min);  % Cutoff protection
    
    % State of charge update
    dQ = (I_total(i) / 3600) * dt;  % Ah consumed
    Capacity_used(i) = Capacity_used(i-1) + dQ;
    SOC(i) = max(0, 1 - Capacity_used(i)/Q_batt);
    
    %% ---- 6. Safety Checks ----
    if SOC(i) < 0.2 && i > 100
        fprintf('WARNING: Battery at 20%% SOC at t=%.1f s\n', t(i));
    end
    if V_batt(i) <= V_min
        fprintf('WARNING: Battery cutoff voltage reached at t=%.1f s\n', t(i));
        break;
    end
end

% Trim arrays if simulation ended early
t = t(1:i);
N = i;

%% ========================================================================
%                           RESULTS & ANALYSIS
% =========================================================================
fprintf('\n========== MISSION SUMMARY ==========\n');
fprintf('Flight Duration:        %.1f min\n', t(end)/60);
fprintf('Final SOC:              %.1f %%\n', SOC(end)*100);
fprintf('Capacity Used:          %.2f Ah (%.0f%% of %.1f Ah)\n', ...
        Capacity_used(end), Capacity_used(end)/Q_batt*100, Q_batt);
fprintf('Energy Consumed:        %.1f Wh\n', trapz(t, Power_elec)/3600);
fprintf('Average Power:          %.1f W\n', mean(Power_elec));
fprintf('Peak Current:           %.1f A (%.0f%% of %dA max)\n', ...
        max(I_total), max(I_total)/I_max*100, I_max);
fprintf('Average Efficiency:     %.1f %%\n', ...
        mean(Power_mech./Power_elec)*100);

% Stall speed check
V_stall = sqrt(2*mass*g/(rho*S*CL_max));
fprintf('\nStall Speed:            %.1f m/s (%.1f mph)\n', ...
        V_stall, V_stall*2.237);
fprintf('Min Flight Speed:       %.1f m/s (%.1f mph)\n', ...
        min(V_aircraft), min(V_aircraft)*2.237);
fprintf('Wing Loading:           %.1f N/m² (%.2f lb/ft²)\n', ...
        mass*g/S, mass*g/S*0.0208854);

% Time to 80% depth of discharge
idx_80 = find(SOC < 0.2, 1);
if ~isempty(idx_80)
    fprintf('Time to 80%% DoD:        %.1f min\n', t(idx_80)/60);
else
    fprintf('Time to 80%% DoD:        >%.1f min (not reached)\n', t(end)/60);
end

%% ========================================================================
%                                 PLOTS
% =========================================================================
figure('Position', [100 100 1200 800]);

% Plot 1: Battery State
subplot(3,2,1);
yyaxis left
plot(t/60, SOC*100, 'LineWidth', 2);
ylabel('State of Charge (%)');
ylim([0 100]);
yyaxis right
plot(t/60, V_batt, 'LineWidth', 2);
ylabel('Battery Voltage (V)');
xlabel('Time (min)');
title('Battery State');
grid on; legend('SOC', 'Voltage', 'Location', 'best');

% Plot 2: Current Draw
subplot(3,2,2);
plot(t/60, I_total, 'LineWidth', 2); hold on;
yline(I_max, 'r--', 'LineWidth', 1.5, 'Label', 'Max Continuous');
ylabel('Total Current (A)');
xlabel('Time (min)');
title('Current Draw (Both Motors)');
grid on; legend('Current', 'Max (80C)', 'Location', 'best');

% Plot 3: Power
subplot(3,2,3);
plot(t/60, Power_elec, 'LineWidth', 2); hold on;
plot(t/60, Power_mech, 'LineWidth', 2);
ylabel('Power (W)');
xlabel('Time (min)');
title('Power Consumption');
grid on; legend('Electrical', 'Mechanical', 'Location', 'best');

% Plot 4: Thrust & Drag
subplot(3,2,4);
plot(t/60, Thrust_avail, 'LineWidth', 2); hold on;
plot(t/60, Drag, 'LineWidth', 2);
ylabel('Force (N)');
xlabel('Time (min)');
title('Thrust vs Drag');
grid on; legend('Thrust Available', 'Drag', 'Location', 'best');

% Plot 5: Flight Performance
subplot(3,2,5);
yyaxis left
plot(t/60, V_aircraft, 'LineWidth', 2);
ylabel('Velocity (m/s)');
yline(V_stall, 'r--', 'LineWidth', 1.5);
yyaxis right
plot(t/60, CL, 'LineWidth', 2);
ylabel('Lift Coefficient');
xlabel('Time (min)');
title('Flight Performance');
grid on; legend('Velocity', 'Stall Speed', 'C_L', 'Location', 'best');

% Plot 6: Motor Performance
subplot(3,2,6);
yyaxis left
plot(t/60, RPM/1000, 'LineWidth', 2);
ylabel('RPM (x1000)');
yyaxis right
plot(t/60, throttle*100, 'LineWidth', 2);
ylabel('Throttle (%)');
xlabel('Time (min)');
title('Motor Operation');
grid on; legend('RPM', 'Throttle', 'Location', 'best');

sgtitle('AIAA DBF Power System Analysis - Twin Motor 6S Configuration', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%                          DESIGN RECOMMENDATIONS
% =========================================================================
fprintf('\n========== DESIGN RECOMMENDATIONS ==========\n');

% Battery margin check
if SOC(end) > 0.3
    fprintf('✓ Good battery margin (%.0f%% remaining)\n', SOC(end)*100);
else
    fprintf('⚠ Low battery margin - consider larger capacity\n');
end

% Thrust margin check
thrust_margin = mean(Thrust_avail(100:end) ./ Thrust_req(100:end));
if thrust_margin > 1.3
    fprintf('✓ Good thrust margin (%.1fx drag in cruise)\n', thrust_margin);
elseif thrust_margin > 1.1
    fprintf('⚠ Adequate thrust margin (%.1fx) - consider more power\n', thrust_margin);
else
    fprintf('✗ Insufficient thrust margin (%.1fx) - INCREASE POWER\n', thrust_margin);
end

% Efficiency check
avg_efficiency = mean(Power_mech./Power_elec)*100;
if avg_efficiency > 65
    fprintf('✓ Good propulsion efficiency (%.1f%%)\n', avg_efficiency);
else
    fprintf('⚠ Low efficiency (%.1f%%) - check prop selection\n', avg_efficiency);
end

fprintf('============================================\n');

%% END SIMULATION