clc
clear
close all

% Simulate the autopilot %
options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5]);
tspan = linspace(0,20,5000);

% Calling sequence: ode45(System function, time length, ICs, options)
[T,x] = ode45(@AircraftPitchSystem,tspan,[0 0 0]',options);
figure
plot(T,x(:,3),'linewidth',2)
xlabel('Time [sec.]','fontsize',16,'interpreter','latex')
ylabel('Pitch $\theta(t)$ with r(t) = 0.2 rad.',...
'fontsize',16,'interpreter','latex')
title('P controller')

%% What exactly does the control u(t) look like? %
for i = 1:length(T)
    [dx,tu] = AircraftPitchSystem(T(i),x(i,:));
    uk(i) = tu;
end

figure
plot(T,uk,'linewidth',2)
xlabel('Time [sec.]','fontsize',16,'interpreter','latex')
ylabel('Control $u(t)$','fontsize',16,'interpreter','latex')

