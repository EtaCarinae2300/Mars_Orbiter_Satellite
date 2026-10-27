clear; close all; clc;

%% Params Definition
params = ParamsDefinition(); 

%% Orbit Definition
% Defines the orbital params of the 2 spacecrafts by computing their 
% position and velocity vectors in the inertial frame at selected time instances

% Convert Orbital Frame to Inertial Frame for LMO & GMO
DCM_O2I_LMO = DCM_O2I(params.LMO, 450); % We are evaluating for a time of 450s
[r_inertial_LMO, v_inertial_LMO] = pos_vel_calc(DCM_O2I_LMO, params.LMO);
display(r_inertial_LMO);
display(v_inertial_LMO);

DCM_O2I_GMO = DCM_O2I(params.GMO, 1150); % We are evaluating for a time of 1150s
[r_inertial_GMO, v_inertial_GMO] = pos_vel_calc(DCM_O2I_LMO, params.LMO);
display(r_inertial_GMO);
display(v_inertial_GMO);

% For future, lets visualize this simulation

%% Numerical Attitude Simulation
% Initializing params for RK4 Integrator
% Time Parameters
dt = 1;
Time = 500;
N = Time/dt;
t = 0:dt:Time;

%% Testing the Simulator with Torque-Free Motion
% Initializing spacecraft attitude and angular velocity
omega_BN_0Torque = zeros(3, length(t));
sigma_BN_0Torque = zeros(3, length(t));

omega_BN_0Torque(:,1) = params.Spacecraft.w_BN;
sigma_BN_0Torque(:,1) = params.Spacecraft.sigma_BN_t0;

u_0Torque = [0; 0; 0];
for i = 1:N
    [sigma_BN_0Torque(:,i+1), omega_BN_0Torque(:,i+1)] = RK4_Integration(sigma_BN_0Torque(:,i), omega_BN_0Torque(:,i), ...
                                                   params.Spacecraft.I_B, u_0Torque, dt);
end

[H_0Torque, T_0Torque] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BN_0Torque);
PlotSimulation(t,sigma_BN_0Torque,omega_BN_0Torque, H_0Torque, T_0Torque)
   
%% Testing the Simulator with Fixed Control Torque
omega_BN_FixedTorque = zeros(3, length(t));
sigma_BN_FixedTorque = zeros(3, length(t));

omega_BN_FixedTorque(:,1) = params.Spacecraft.w_BN;
sigma_BN_FixedTorque(:,1) = params.Spacecraft.sigma_BN_t0;

u_FixedTorque = [0.01,-0.01, 0.02]';
for i = 1:N
    [sigma_BN_FixedTorque(:,i+1), omega_BN_FixedTorque(:,i+1)] = RK4_Integration(sigma_BN_FixedTorque(:,i), omega_BN_FixedTorque(:,i), ...
            params.Spacecraft.I_B, u_FixedTorque, dt);
end
[H_FixedTorque, T_FixedTorque] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BN_FixedTorque);
PlotSimulation(t,sigma_BN_FixedTorque,omega_BN_FixedTorque, H_FixedTorque, T_FixedTorque)

%% Testing the Simulator For Variable Control Torque
Ti = 120; % Max Decay Response time
zeta = 1; % Damping Coefficient (Critically Damped = 1)
P = (2*max(diag(params.Spacecraft.I_B)))/Ti;
K = (min(diag(params.Spacecraft.I_B)) * ((zeta/P)^2))^-1;

%% Testing the Simulator for Sun-Pointing Control

% Initializing omegaBsN & sigmaBsN
omega_BsN = zeros(3, length(t));
sigma_BsN = zeros(3, length(t));
sigma_BsR = zeros(3, length(t));
w_BsR = zeros(3, length(t));
u_S = zeros(3,length(t));

[RsN, w_RsN] = Calc_PointingFrame('Sun-Pointing', params, Time);

omega_BsN(:,1) = params.Spacecraft.w_BN;
sigma_BsN(:,1) = params.Spacecraft.sigma_BN_t0;
[sigma_BsR(:,1), w_BsR(:,1)] = Calc_AttitudeError(MRP2dcm(sigma_BsN(:,1)), omega_BsN(:,1), RsN, w_RsN);

for i = 1:N
    u_S(:,i) = (-K*sigma_BsR(:,i)) - (P*w_BsR(:,i));
    [sigma_BsN(:,i+1), omega_BsN(:,i+1)] = RK4_Integration(sigma_BsN(:,i), omega_BsN(:,i), ...
        params.Spacecraft.I_B, u_S(:,i), dt);
    [sigma_BsR(:,i+1), w_BsR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BsN(:,i+1)), omega_BsN(:,i+1), RsN, w_RsN);
end

[H_SunPointing, T_SunPointing] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BsN);
PlotSimulation(t, sigma_BsR, w_BsR, H_SunPointing, T_SunPointing, u_S);

%% Testing the Simulator for Nadir-Pointing Control

% Initializing omegaBsN & sigmaBsN
omega_BnN = zeros(3, length(t));
sigma_BnN = zeros(3, length(t));
sigma_BnR = zeros(3, length(t));
w_BnR = zeros(3, length(t));
u_N = zeros(3,length(t));

[RnN, w_RnN] = Calc_PointingFrame('Nadir-Pointing', params, t(1));

omega_BnN(:,1) = params.Spacecraft.w_BN;
sigma_BnN(:,1) = params.Spacecraft.sigma_BN_t0;
[sigma_BnR(:,1), w_BnR(:,1)] = Calc_AttitudeError(MRP2dcm(sigma_BnN(:,1)), omega_BnN(:,1), RnN, w_RnN);

for i = 1:N
    u_N(:,i) = (-K*sigma_BnR(:,i)) - (P*w_BnR(:,i));
    [sigma_BnN(:,i+1), omega_BnN(:,i+1)] = RK4_Integration(sigma_BnN(:,i), omega_BnN(:,i), ...
        params.Spacecraft.I_B, u_N(:,i), dt);
    [RnN, w_RnN] = Calc_PointingFrame('Nadir-Pointing', params, t(i+1));
    [sigma_BnR(:,i+1), w_BnR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BnN(:,i+1)), omega_BnN(:,i+1), RnN, w_RnN);
end

[H_SunPointing, T_SunPointing] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BnN);
PlotSimulation(t, sigma_BnR, w_BnR, H_SunPointing, T_SunPointing, u_N);

%% Testing the Simulator for GMO-Pointing Control

% Initializing omegaBsN & sigmaBsN
omega_BgN = zeros(3, length(t));
sigma_BgN = zeros(3, length(t));
sigma_BgR = zeros(3, length(t));
w_BgR = zeros(3, length(t));
u_G = zeros(3,length(t));

[RgN, w_RgN] = Calc_PointingFrame('GMO-Pointing', params, t(1));

omega_BgN(:,1) = params.Spacecraft.w_BN;
sigma_BgN(:,1) = params.Spacecraft.sigma_BN_t0;
[sigma_BgR(:,1), w_BgR(:,1)] = Calc_AttitudeError(MRP2dcm(sigma_BgN(:,1)), omega_BgN(:,1), RgN, w_RgN);

for i = 1:N
    u_G(:,i) = (-K*sigma_BgR(:,i)) - (P*w_BgR(:,i));
    [sigma_BgN(:,i+1), omega_BgN(:,i+1)] = RK4_Integration(sigma_BgN(:,i), omega_BgN(:,i), ...
        params.Spacecraft.I_B, u_G(:,i), dt);
    [RgN, w_RgN] = Calc_PointingFrame('GMO-Pointing', params, t(i+1));
    [sigma_BgR(:,i+1), w_BgR(:,i+1)] = Calc_AttitudeError(MRP2dcm(sigma_BgN(:,i+1)), omega_BgN(:,i+1), RgN, w_RgN);
end

[H_SunPointing, T_SunPointing] = Calc_AngMom_AngKE(params.Spacecraft.I_B, omega_BgN);
PlotSimulation(t, sigma_BgR, w_BgR, H_SunPointing, T_SunPointing, u_G);






