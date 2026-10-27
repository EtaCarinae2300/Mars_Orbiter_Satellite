clear; close all; clc;

%% Params Definition
params = ParamsDefinition(); 

%% Mission Scenario Simulation

% Initializing params for RK4 Integrator
% Time Parameters
dt = 1;
Time = 6500;
N = Time/dt;
t = 0:dt:Time;

% Initializing Mission Scenario Variables
mission_scene = zeros(1, length(t));
in_sun_check = zeros(1, length(t));
LMO_GMO_angle = zeros(1, length(t));

% Defining Gains for the Control Law
Ti = 120; % Max Decay Response time
zeta = 1; % Damping Coefficient (Critically Damped = 1)
P = (2*max(diag(params.Spacecraft.I_B)))/Ti; % Derivative Gain
K = (min(diag(params.Spacecraft.I_B)) * ((zeta/P)^2))^-1; % Proportionality Gain

% Initializing MRPs & Angular Rate Variables
omega_BN = zeros(3, length(t));
sigma_BN = zeros(3, length(t));
sigma_BR = zeros(3, length(t));
w_BR = zeros(3, length(t));
u = zeros(3,length(t));

[RN, w_RN] = Calc_PointingFrame('Sun-Pointing', params, t(1));

omega_BN(:,1) = params.Spacecraft.w_BN;
sigma_BN(:,1) = params.Spacecraft.sigma_BN_t0;
[sigma_BR(:,1), w_BR(:,1)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,1)), omega_BN(:,1), RN, w_RN);


for i = 1:N
    
    % Mission Operating Scenario Check
    DCM_O2I_LMO = DCM_O2I(params.LMO, t(i));
    [r_inertial_LMO, v_inertial_LMO] = pos_vel_calc(DCM_O2I_LMO, params.LMO);
    
    in_sun_check(:,i) = r_inertial_LMO(2);
    
    DCM_O2I_GMO = DCM_O2I(params.GMO, t(i));
    [r_inertial_GMO, v_inertial_GMO] = pos_vel_calc(DCM_O2I_GMO, params.GMO);

    LMO_GMO_angle(:,i) = acosd(dot((r_inertial_LMO/norm(r_inertial_LMO)),(r_inertial_GMO/norm(r_inertial_GMO))));
    
    if in_sun_check(:,i) > 0
    	mission_scene(:,i) = 1;
    elseif LMO_GMO_angle(i) > 35
    	mission_scene(:,i) = 2;
    else
        mission_scene(:,i) = 3;
    end
    
    if mission_scene(i) == 1
        [RN, w_RN] = Calc_PointingFrame('Sun-Pointing', params, t(i));
        [sigma_BR(:,i), w_BR(:,i)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i)), omega_BN(:,i), RN, w_RN);
        u(:,i) = (-K*sigma_BR(:,i)) - (P*w_BR(:,i));

        [sigma_BN(:,i+1), omega_BN(:,i+1)] = RK4_Integration(sigma_BN(:,i), omega_BN(:,i), ...
            params.Spacecraft.I_B, u(:,i), dt);
        [sigma_BR(:,i+1), w_BR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i+1)), omega_BN(:,i+1), RN, w_RN);
    
    elseif mission_scene(i) == 2
        [RN, w_RN] = Calc_PointingFrame('Nadir-Pointing', params, t(i));
        [sigma_BR(:,i), w_BR(:,i)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i)), omega_BN(:,i), RN, w_RN);
        u(:,i) = -K*sigma_BR(:,i) - P*w_BR(:,i);

        [sigma_BN(:,i+1), omega_BN(:,i+1)] = RK4_Integration(sigma_BN(:,i), omega_BN(:,i), ...
            params.Spacecraft.I_B, u(:,i), dt);
        [RN, w_RN] = Calc_PointingFrame('Nadir-Pointing', params, t(i+1));
        [sigma_BR(:,i+1), w_BR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i+1)), omega_BN(:,i+1), RN, w_RN);
    
    elseif mission_scene(i) == 3
        [RN, w_RN] = Calc_PointingFrame('GMO-Pointing', params, t(i));
        [sigma_BR(:,i), w_BR(:,i)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i)), omega_BN(:,i), RN, w_RN);
        u(:,i) = -K*sigma_BR(:,i) - P*w_BR(:,i);

        [sigma_BN(:,i+1), omega_BN(:,i+1)] = RK4_Integration(sigma_BN(:,i), omega_BN(:,i), ...
            params.Spacecraft.I_B, u(:,i), dt);
        [RN, w_RN] = Calc_PointingFrame('GMO-Pointing', params, t(i+1));
        [sigma_BR(:,i+1), w_BR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BN(:,i+1)), omega_BN(:,i+1), RN, w_RN);
    end
end

[H_Mission, T_Mission] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BN);

plot(mission_scene)
PlotSimulation(t, sigma_BR, w_BR, H_Mission, T_Mission, u);





