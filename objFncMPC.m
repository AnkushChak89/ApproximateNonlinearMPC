function J = objFncMPC(u, x, MPC)

% Ankush Chakrabarty (chakank@me.com)

J = 0;
for k = 1:MPC.Np
    x = model(x, u(k));                 % update state via nonlinear model
    J = J + x.'* MPC.Q * x ...
          + u(k).'* MPC.R * u(k);       % stage cost
    if constraint_violated(x, MPC)
        J = J + 10^4;
    end
end
J = J + x.' * MPC.P * x;                % terminal cost