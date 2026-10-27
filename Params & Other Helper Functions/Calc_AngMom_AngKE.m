function [H, T] = Calc_AngMom_AngKE(I_B, omega)
% Calculates angular momentum and rotational kinetic energy.
% Inputs: I_B - Inertia tensor, omega - Angular velocity history [rad/s]
% Outputs: H - Angular momentum history, T - Rotational kinetic energy history

    % Calculate angular momentum
    H = I_B * omega;

    % Calculate rotational kinetic energy
    T = zeros(1, size(omega, 2));

    for i = 1:size(omega, 2)
        w = omega(:,i);
        T(i) = 0.5 * w' * I_B * w;
    end

end