function [RN, w_RN] = Calc_PointingFrame(mode, params, tsim)
% Calculates the reference frame orientation and angular velocity for a given pointing mode.
% Modes: 'Sun-Pointing', 'Nadir-Pointing', 'GMO-Pointing'
% Inputs: mode - Pointing mode, params - Spacecraft/Orbit Parameters, tsim - Simulation time [s]
% Outputs: RN - Reference frame DCM, w_RN - Reference frame angular velocity [rad/s]

switch mode

    case 'Sun-Pointing'
        % Sun-pointing reference frame is fixed in inertial space
        RN = [-1, 0, 0;
               0, 0, 1;
               0, 1, 0];

        w_RN = [0; 0; 0];

    case 'Nadir-Pointing'
        % Define 180-degree rotation about the second axis
        DCM180 = [-1, 0, 0;
                   0, 1, 0;
                   0, 0, -1];

        % Calculate Hill frame orientation at simulation time
        HN = Calc_HillFrame(params.LMO, tsim);

        % Calculate Nadir-pointing reference frame orientation and angular velocity
        RN = DCM180 * HN;
        w_RN = HN' * [0; 0; params.LMO.w];

    case 'GMO-Pointing'
        % Define time step for numerical angular velocity calculation
        dt = 0.00001;

        % Calculate LMO and GMO inertial positions at current time
        DCM_LMO = DCM_O2I(params.LMO, tsim);
        [r_LMO, ~] = pos_vel_calc(DCM_LMO, params.LMO);

        DCM_GMO = DCM_O2I(params.GMO, tsim);
        [r_GMO, ~] = pos_vel_calc(DCM_GMO, params.GMO);

        % Calculate relative position vector from LMO to GMO
        del_r = r_GMO - r_LMO;

        % Define GMO-pointing reference frame unit vectors
        r1 = -del_r / norm(del_r);
        r2 = cross(del_r, [0; 0; 1]) / norm(cross(del_r, [0; 0; 1]));
        r3 = cross(r1, r2);

        % Construct reference frame DCM at current time
        RN = [r1, r2, r3]';

        % Calculate LMO and GMO inertial positions at next time step
        DCM_LMO_dt = DCM_O2I(params.LMO, tsim + dt);
        [r_LMO_dt, ~] = pos_vel_calc(DCM_LMO_dt, params.LMO);

        DCM_GMO_dt = DCM_O2I(params.GMO, tsim + dt);
        [r_GMO_dt, ~] = pos_vel_calc(DCM_GMO_dt, params.GMO);

        % Calculate updated relative position vector
        del_r_dt = r_GMO_dt - r_LMO_dt;

        % Define updated GMO-pointing reference frame unit vectors
        r1 = -del_r_dt / norm(del_r_dt);
        r2 = cross(del_r_dt, [0; 0; 1]) / norm(cross(del_r_dt, [0; 0; 1]));
        r3 = cross(r1, r2);

        % Construct reference frame DCM at next time step
        RcN_dt = [r1, r2, r3]';

        % Calculate reference frame angular velocity using numerical differentiation
        w_RN_skew = -((RcN_dt - RN) / dt) * RN';

        % Extract angular velocity vector from skew-symmetric matrix
        w_RN = RN' * [w_RN_skew(3,2);
                      w_RN_skew(1,3);
                      w_RN_skew(2,1)];

    otherwise
        error('Invalid pointing mode: %s', mode);

end

end
