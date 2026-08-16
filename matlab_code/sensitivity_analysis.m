function sensitivity_analysis(params)
% Drone Flight Simulation Project
% Sensitivity Analysis
% Investigates how mass, drag and pitch affect the drones velocities.

figure('Name','Sensitivity Analysis',...
       'NumberTitle','off',...
       'Position',[100 50 1200 900]);

%% MASS 
massValues = [1.0 3.0];

for j = 1:length(massValues)

    p = params;
    p.mass = massValues(j);

    [t,Y] = analytical_solution(p);

    vxMass(j,:) = Y(4,:);
    vyMass(j,:) = Y(5,:);
    vzMass(j,:) = Y(6,:);

end

%% DRAG 
dragValues = [0.5 1.0];

for j = 1:length(dragValues)

    p = params;
    p.drag = dragValues(j);

    [t,Y] = analytical_solution(p);

    vxDrag(j,:) = Y(4,:);
    vyDrag(j,:) = Y(5,:);
    vzDrag(j,:) = Y(6,:);

end

%% PITCH
pitchValues = [10 25];

for j = 1:length(pitchValues)

    p = params;
    p.pitch = deg2rad(pitchValues(j));

    [t,Y] = analytical_solution(p);

    vxPitch(j,:) = Y(4,:);
    vyPitch(j,:) = Y(5,:);
    vzPitch(j,:) = Y(6,:);

end

%% MASS 

subplot(3,3,1)
plot(t,vxMass(1,:),'k','LineWidth',2)
hold on
plot(t,vxMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('1 kg','3 kg')

subplot(3,3,2)
plot(t,vyMass(1,:),'k','LineWidth',2)
hold on
plot(t,vyMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('1 kg','3 kg')

subplot(3,3,3)
plot(t,vzMass(1,:),'k','LineWidth',2)
hold on
plot(t,vzMass(2,:),'m--','LineWidth',2)
grid on
title('Mass: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('1 kg','3 kg')

%% DRAG

subplot(3,3,4)
plot(t,vxDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vxDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('k = 0.5','k = 1.0')

subplot(3,3,5)
plot(t,vyDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vyDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('k = 0.5','k = 1.0')

subplot(3,3,6)
plot(t,vzDrag(1,:),'k','LineWidth',2)
hold on
plot(t,vzDrag(2,:),'r--','LineWidth',2)
grid on
title('Drag: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('k = 0.5','k = 1.0')

%% PITCH 

subplot(3,3,7)
plot(t,vxPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vxPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Forward Velocity')
xlabel('Time (s)')
ylabel('v_x (m/s)')
legend('10°','25°')

subplot(3,3,8)
plot(t,vyPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vyPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Lateral Velocity')
xlabel('Time (s)')
ylabel('v_y (m/s)')
legend('10°','25°')

subplot(3,3,9)
plot(t,vzPitch(1,:),'k','LineWidth',2)
hold on
plot(t,vzPitch(2,:),'b--','LineWidth',2)
grid on
title('Pitch: Vertical Velocity')
xlabel('Time (s)')
ylabel('v_z (m/s)')
legend('10°','25°')

%% Print summary

fprintf('\n PARAMETER STUDY \n');

fprintf('\nMass\n');
fprintf('1 kg Final Speed : %.2f m/s\n',norm([vxMass(1,end),vyMass(1,end),vzMass(1,end)]));
fprintf('3 kg Final Speed : %.2f m/s\n',norm([vxMass(2,end),vyMass(2,end),vzMass(2,end)]));

fprintf('\nDrag\n');
fprintf('k = 0.5 Final Speed : %.2f m/s\n',norm([vxDrag(1,end),vyDrag(1,end),vzDrag(1,end)]));
fprintf('k = 1.0 Final Speed : %.2f m/s\n',norm([vxDrag(2,end),vyDrag(2,end),vzDrag(2,end)]));

fprintf('\nPitch\n');
fprintf('10 deg Final Speed : %.2f m/s\n',norm([vxPitch(1,end),vyPitch(1,end),vzPitch(1,end)]));
fprintf('25 deg Final Speed : %.2f m/s\n',norm([vxPitch(2,end),vyPitch(2,end),vzPitch(2,end)]));

fprintf('\n');

end