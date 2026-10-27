
function PlotSimulation(t, sigma_BN, omega_BN, H, T, u)
% Plots spacecraft attitude, angular velocity, and other simulation results.
% Inputs: t - Time vector [s], sigma_BN - Attitude MRPs, omega_BN - Angular velocity [rad/s]
% Optional: H - Angular momentum, T - Rotational kinetic energy, u - Control torque

% Plot spacecraft attitude MRPs
figure;
plot(t, sigma_BN');
xlabel('Time [s]');
ylabel('MRP');
legend('\sigma_1', '\sigma_2', '\sigma_3');
title('Spacecraft Attitude');
grid on;

% Plot spacecraft angular velocity
figure;
plot(t, omega_BN');
xlabel('Time [s]');
ylabel('\omega [rad/s]');
legend('\omega_x', '\omega_y', '\omega_z');
title('Spacecraft Angular Velocity');
grid on;

% Plot angular momentum if provided
if nargin >= 4 && ~isempty(H)
    figure;
    plot(t, H');
    xlabel('Time [s]');
    ylabel('Angular Momentum [kg m^2/s]');
    legend('H_x', 'H_y', 'H_z');
    title('Spacecraft Angular Momentum');
    grid on;
end

% Plot rotational kinetic energy if provided
if nargin >= 5 && ~isempty(T)
    figure;
    plot(t, T);
    xlabel('Time [s]');
    ylabel('Rotational Kinetic Energy [J]');
    title('Spacecraft Rotational Kinetic Energy');
    grid on;
end

% Plot control torque if provided
if nargin >= 6 && ~isempty(u)
    figure;
    plot(t, u');
    xlabel('Time [s]');
    ylabel('Control Torque [Nm]');
    legend('u_x', 'u_y', 'u_z');
    title('Spacecraft Control Torque');
    grid on;
end

end
