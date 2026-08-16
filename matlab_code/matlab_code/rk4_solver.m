function [t, Y] = rk4_solver(ode_fun, params, ctrl_mode)
% =========================================================================
% Classical 4th-Order Runge-Kutta Method (RK4 Solver)
% Local Truncation Error: O(h^5), Global Error: O(h^4)
% =========================================================================

if nargin < 3
    ctrl_mode = 'itae';
end

t = params.t0:params.dt:params.tf;
N = length(t);
h = params.dt;
num_states = length(params.Y0);

Y = zeros(num_states, N);
Y(:, 1) = params.Y0;

for n = 1:N-1
    tn = t(n);
    yn = Y(:, n);
    
    k1 = ode_fun(tn, yn, params, ctrl_mode);
    k2 = ode_fun(tn + 0.5*h, yn + 0.5*h*k1, params, ctrl_mode);
    k3 = ode_fun(tn + 0.5*h, yn + 0.5*h*k2, params, ctrl_mode);
    k4 = ode_fun(tn + h, yn + h*k3, params, ctrl_mode);
    
    Y(:, n+1) = yn + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
end

end
