function [t, Y] = euler_solver(ode_fun, params, ctrl_mode)
% =========================================================================
% Forward Euler Method (First-Order Explicit Solver)
% Formula: Y_{n+1} = Y_n + h * f(t_n, Y_n)
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
    dY = ode_fun(t(n), Y(:, n), params, ctrl_mode);
    Y(:, n+1) = Y(:, n) + h * dY;
end

end
