%%
%QUESTION 1(b)
%Solving analytically
syms t
syms v(t)
ode = diff(v,t) == g - (c/m)*v;       % dv/dt = g - (c/m)*v
vSol = dsolve(ode, v(0)==v0);
vSol = simplify(vSol)  ;               % display symbolic expression
disp(vSol)
%%
%Using ODE45 solver to solve an equation: m(dv/dt) + cv = mg;
v0 = 0;
tRange = 0:1:10;
m = 5.0;
c = 2.0;
g = 9.8;
%Differential equation
V_t = @(t, v) (m * g - c * v) / m;


% Solve the ODE using ode45
[t, v] = ode45(V_t, tRange, v0);
%Graphing the result,
plot(t, v, 'Marker', 'o');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs Time');
grid on
%Tabulating the results
results = table(t, v, 'VariableNames', {'Time', 'Velocity'});
disp(results);

%Solving numerically
h = 1;
numsteps = length(tRange);
%Using Euler's method
V_euler = zeros(numsteps, 1);
V_euler(1) = v0;
for i = 1:numsteps-1
    V_euler(i+1) = V_euler(i) + h * V_t(t(i), V_euler(i));
end
%Display results
% Display results for Euler method
relativeErrorEuler = abs((V_euler - v) ./ v);
tab_results_euler = table(t, v, V_euler, relativeErrorEuler, 'VariableNames', {'Time', 'Actual Velocity', 'EulerVelocity', 'RelativeError'});
disp(tab_results_euler);

%Using improved Euler (Heun) method
V_heun = zeros(numsteps, 1);
V_heun(1) = v0;

for i = 1:numsteps-1
    slope1 = V_t(t(i), V_heun(i)); 
    %Predictor term
    v_predict = V_heun(i) + h * slope1;            
    slope2 = V_t(t(i+1), v_predict);  
    %Corrector term
    V_heun(i+1) = V_heun(i) + (h/2) * (slope1 + slope2); 
end

%Display results for Heun method
% Display results for Heun method
relativeErrorHeun = abs((V_heun - v) ./ v);
tab_results_heun = table(t, v, V_heun, relativeErrorHeun, 'VariableNames', {'Time', 'Actual Velocity', 'HeunVelocity', 'RelativeError'});
disp(tab_results_heun);

%Using RK-4 method to solve the ode
V_rk4 = zeros(numsteps, 1);
V_rk4(1) = v0;
for i = 1:numsteps-1
 k1 = V_t(t(i), V_rk4(i));
 k2 = V_t(t(i) + h/2, V_rk4(i) + h*k1/2);
 k3 = V_t(t(i) + h/2, V_rk4(i) + h*k2/2);
 k4 = V_t(t(i) + h, V_rk4(i) + h*k3);
 V_rk4(i+1) = V_rk4(i) + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end
%Display results
% Display results for RK-4 method
relativeErrorRK4 = abs((V_rk4 - v) ./ v);
tab_results_rk4 = table(t, v, V_rk4, relativeErrorRK4, 'VariableNames', {'Time', 'Actual Velocity' 'RK4Velocity', 'RelativeError'});
disp(tab_results_rk4);

%Plot all methods on one graph
figure;
hold on; 
plot(t, v, 'Marker', 'o', 'DisplayName', 'Velocity');
plot(t, V_euler, 'x-', 'DisplayName', 'Euler Method');
plot(t, V_heun, 's-', 'DisplayName', 'Heun Method');
plot(t, V_rk4, 'd-', 'DisplayName', 'RK-4 Method');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Comparison of Numerical Methods for Velocity');
legend('show');
grid on;
hold off;


% Solving a second-order differential equation using MATLAB (ode45)
% The equation is first converted into two first-order equations:

% dy/dt = z
% dz/dt = (1-y^2)z - y
f = @(t,Y) [Y(2);
    (1-Y(1)^2)*Y(2)-Y(1)]; % u(1)=y, u(2)=y'

t = 0:0.2:10; %range of values
Y0 = [1;1]; % Initial conditions [y(0); z(0)] where z = y'

[t,Y] = ode45(f,t,Y0); 

Y = round(Y,2);
T = table(t,Y(:,1),'VariableNames',{'t','y'});

disp(T)
