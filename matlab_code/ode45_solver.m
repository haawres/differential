function [t, Y] = ode45_solver(ode_fun, params, ctrl_mode)
% =========================================================================
% MATLAB ode45 Solver Wrapper (Explicit Runge-Kutta (4,5) Formula)
% Adaptive Step-Size Numerical Integration
% =========================================================================

if nargin < 3
    ctrl_mode = 'itae';
end

tspan = [params.t0 params.tf];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

[t, Y_out] = ode45(@(t, y) ode_fun(t, y, params, ctrl_mode), tspan, params.Y0, options);

% Output as state rows for consistency
Y = Y_out';

end
