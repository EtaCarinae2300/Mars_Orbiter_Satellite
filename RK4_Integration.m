function [sigma_next, w_next] = RK4_Integration(sigma, w, I_B, u, dt)
% Propagates spacecraft attitude and angular velocity using RK4 integration.
% Inputs: sigma - Current MRP, w - Angular velocity [rad/s]
%         I_B - Spacecraft inertia tensor, u - Control torque [Nm], dt - Time step [s]
% Outputs: sigma_next - Updated MRP, w_next - Updated angular velocity [rad/s]

% Define rotational dynamics
f_omega = @(w) I_B \ (-cross(w, I_B*w) + u);

% Define MRP kinematics
f_sigma = @(sig, w) 0.25 * ...
    ((1 - sig'*sig)*eye(3) + 2*vec2skew(sig) + 2*(sig*sig')) * w;

% RK4 integration for angular velocity
k1 = f_omega(w);
k2 = f_omega(w + 0.5*dt*k1);
k3 = f_omega(w + 0.5*dt*k2);
k4 = f_omega(w + dt*k3);

w_next = w + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

% RK4 integration for attitude MRPs
k1s = f_sigma(sigma, w);
k2s = f_sigma(sigma + 0.5*dt*k1s, w + 0.5*dt*k1);
k3s = f_sigma(sigma + 0.5*dt*k2s, w + 0.5*dt*k2);
k4s = f_sigma(sigma + dt*k3s, w + dt*k3);

sigma_next = sigma + (dt/6)*(k1s + 2*k2s + 2*k3s + k4s);

% Apply MRP shadow set when necessary
if norm(sigma_next) > 1
    sigma_next = -sigma_next / norm(sigma_next)^2;
end

end
