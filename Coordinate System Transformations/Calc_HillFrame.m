function HN = Calc_HillFrame(orbit, t_sim)
% Calculates the Hill frame orientation relative to the inertial frame.
% Inputs: orbit - Orbital parameters struct, t_sim - Simulation time [s]
% Output: HN - Inertial-to-Hill frame DCM

    % Calculate orbital-to-inertial DCM at simulation time
    DCM_O2I_LMO = DCM_O2I(orbit, t_sim);

    % Calculate spacecraft inertial position and velocity
    [r_inertial_LMO, v_inertial_LMO] = pos_vel_calc(DCM_O2I_LMO, orbit);

    % Calculate Hill frame unit vectors
    ir = r_inertial_LMO / norm(r_inertial_LMO);
    ih = cross(r_inertial_LMO, v_inertial_LMO) / norm(cross(r_inertial_LMO, v_inertial_LMO));
    itheta = cross(ih, ir);

    % Construct Hill frame DCM
    NH = [ir, itheta, ih];
    HN = NH';

end
