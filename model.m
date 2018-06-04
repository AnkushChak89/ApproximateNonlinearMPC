function xnew = model(x, u)

% Ankush Chakrabarty (chakank@me.com)

% Nonlinear model update
f = [x(2) + u*(0.5 + 0.5*x(1));...
     x(1) + u*(0.5 - 2.0*x(2))];
 
dt = 0.1;                   % sampling time
xnew = x(:) + dt*f(:);      % state update