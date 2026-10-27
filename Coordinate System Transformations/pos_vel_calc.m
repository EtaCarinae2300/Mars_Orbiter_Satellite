function [pos_inertial, vel_inertial] = pos_vel_calc(DCM_O2I, orbit)
% Calculates the spacecraft position and velocity in the inertial frame.
% Inputs: DCM_O2I - Orbital frame DCM, orbit - Orbital parameters struct
% Outputs: pos_inertial - Inertial position [km], vel_inertial - Inertial velocity [km/s]

    % Calculate inertial position vector
    pos_inertial = DCM_O2I' * [orbit.r; 0; 0];

    % Calculate inertial velocity vector
    vel_inertial = DCM_O2I' * [0; orbit.w * orbit.r; 0];

end
