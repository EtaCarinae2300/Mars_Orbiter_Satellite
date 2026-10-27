function DCM_O2I = DCM_O2I(orbit, t_sim)
% Calculates the orbital frame DCM at a given simulation time.
% Inputs: orbit - Orbital parameters struct, t_sim - Simulation time [s]
% Output: DCM_O2I - Inertial-to-orbital frame DCM

    % Convert orbital orientation angles from degrees to radians
    theta0 = deg2rad(orbit.theta);
    i = deg2rad(orbit.i);
    omega = deg2rad(orbit.omega);

    % Calculate current true anomaly
    theta = theta0 + orbit.w * t_sim;

    % Define 3-1-3 rotation matrices
    M3_theta = [cos(theta), sin(theta), 0;
               -sin(theta), cos(theta), 0;
                0, 0, 1];

    M1_i = [1, 0, 0;
            0, cos(i), sin(i);
            0, -sin(i), cos(i)];

    M3_omega = [cos(omega), sin(omega), 0;
               -sin(omega), cos(omega), 0;
                0, 0, 1];

    % Construct inertial-to-orbital frame DCM
    DCM_O2I = M3_theta * M1_i * M3_omega;

end
