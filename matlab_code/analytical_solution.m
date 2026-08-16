function [t, z_ana, vz_ana] = analytical_solution(params, ctrl_mode)
% =========================================================================
% Analytical Solution for Linearized Altitude Closed-Loop ODE
% MATH221 & CE122 Requirement: Closed-form solution to 2nd-order system
%
% Linearized System:
%   m * d2z/dt2 + kd_z * dz/dt + kp_z * z = kp_z * z_target
% Standard Form:
%   d2z/dt2 + 2*zeta*wn * dz/dt + wn^2 * z = wn^2 * z_target
% =========================================================================

if nargin < 2
    ctrl_mode = 'itae';
end

m = params.mass;
z_target = params.target_z;

if strcmpi(ctrl_mode, 'itae')
    kp = params.pid.itae.alt_kp;
    kd = params.pid.itae.alt_kd;
else
    kp = params.pid.ise.alt_kp;
    kd = params.pid.ise.alt_kd;
end

% Second-order natural frequency and damping ratio
wn = sqrt(kp);
zeta = kd / (2 * sqrt(kp));

% Time vector
t = params.t0:params.dt:params.tf;
N = length(t);
z_ana = zeros(1, N);
vz_ana = zeros(1, N);

if zeta < 1.0
    % Underdamped Response (0 < zeta < 1)
    wd = wn * sqrt(1 - zeta^2);
    phi_phase = atan(sqrt(1 - zeta^2) / zeta);
    
    for k = 1:N
        tk = t(k);
        decay = exp(-zeta * wn * tk);
        z_ana(k) = z_target * (1 - (decay / sqrt(1 - zeta^2)) * sin(wd * tk + phi_phase));
        vz_ana(k) = (z_target * wn / sqrt(1 - zeta^2)) * decay * sin(wd * tk);
    end
elseif abs(zeta - 1.0) < 1e-5
    % Critically Damped Response (zeta = 1)
    for k = 1:N
        tk = t(k);
        decay = exp(-wn * tk);
        z_ana(k) = z_target * (1 - (1 + wn * tk) * decay);
        vz_ana(k) = z_target * (wn^2 * tk) * decay;
    end
else
    % Overdamped Response (zeta > 1)
    s1 = -wn * (zeta - sqrt(zeta^2 - 1));
    s2 = -wn * (zeta + sqrt(zeta^2 - 1));
    C1 = s2 / (s1 - s2);
    C2 = -s1 / (s1 - s2);
    
    for k = 1:N
        tk = t(k);
        z_ana(k) = z_target * (1 + C1*exp(s1*tk) + C2*exp(s2*tk));
        vz_ana(k) = z_target * (C1*s1*exp(s1*tk) + C2*s2*exp(s2*tk));
    end
end

end
