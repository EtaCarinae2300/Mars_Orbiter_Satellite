function params = DefineParams()
    %% Mars Characteristic Definitions
    params.Mars.mu = 42828.3; % km^3/s^2
    params.Mars.R = 3396.19;  % km
    
    %% Low Mars Orbit (LMO) Characteristics
    params.LMO.h = 400; % km
    params.LMO.r = params.Mars.R + params.LMO.h; % km
    params.LMO.w = sqrt(params.Mars.mu / params.LMO.r^3); % rad/s
    
    %% Geosynchronous Mars Orbit (GMO) Characteristics
    params.GMO.r = 20424.2; % km
    params.GMO.w = sqrt(params.Mars.mu / params.GMO.r^3); % rad/s
    
    %% Spacecraft Attitude States
    params.Spacecraft.sigma_BN_t0 = [0.3; -0.4; 0.5];
    
    params.Spacecraft.w_BN_deg = [1; 1.75; -2.2]; % deg/s
    params.Spacecraft.w_BN = deg2rad(params.Spacecraft.w_BN_deg); % rad/s
    
    params.Spacecraft.I_B = [10 0 0;
                             0 5 0;
                             0 0 7.5]; % kg m^2
    
    %% Mission Orbit Details
    
    % LMO orbital frame orientation angles
    params.LMO.omega = 20; % deg
    params.LMO.i = 30;     % deg
    params.LMO.theta = 60; % deg
    
    % GMO orbital frame orientation angles
    params.GMO.omega = 0;   % deg
    params.GMO.i = 0;       % deg
    params.GMO.theta = 250; % deg

end
