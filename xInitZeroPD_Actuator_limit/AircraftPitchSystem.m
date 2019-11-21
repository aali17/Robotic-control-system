function [dx,ut] = AircraftPitchSystem(t,x)
% Model of the aircraft system and Control gains %
kp = 5; % Proportional control gain %
kd = 1000; % Dirivative control gain (recall this has ? t wrapped in) %
ki = 0.1; % Integral control gain (recall this has ? t wrapped in) %
r = 0.2; % Desired pitch angle %
e = r - x(3); % Pitch error %

% Account for actuator saturation %
u_limit = pi/4;

% PID regulation - need to keep a running total of the error
% as well as the old error for the derivative term %
% ek is the total error accumulated, e_old is the previous error
persistent ek e_old
edot = kd*(e-e_old);
eint = ki*e;
epro = kp*e;

% First time through e_old will be empty because there's no
% intitialization (need to account for that)
if(isempty(e_old))
   u = 0;
else
   u = edot + epro;
end

% Check and limit actuators here %
if u >= u_limit 
    u = u_limit;
elseif u <= -u_limit
    u = -u_limit;
end

% Keep old values for control %
e_old = e;
ek = [ek;e];

% Enter system dynamics here %
% with no control input
% dx = zeros(3,1);
% dx(1) = -0.313*x(1) + 56.7*x(2);
% dx(2) = -0.0139*x(1) - 0.426*x(2);
% dx(3) = 56.7*x(2);

% with control input
dx = zeros(3,1);
dx(1) = -0.313*x(1) + 56.7*x(2) + 0.232*u;
dx(2) = -0.0139*x(1) - 0.426*x(2) + 0.0203*u;
dx(3) = 56.7*x(2);

% Need this to get the control signal out later %
if nargout>1
   ut = u;
end
 
 