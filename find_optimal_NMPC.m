function [uk, feas] = find_optimal_NMPC(xk, MPC)

% Ankush Chakrabarty (chakank@me.com)
x0 = xk(:);

% Solve the finite-horizon optimal control problem
u_opt = GODLIKE(@(u) objFncMPC(u, x0, MPC),...
                MPC.Ulb*ones(MPC.Np,1), MPC.Uub*ones(MPC.Np,1));
uk = u_opt(1:MPC.Nu);
            
% Check if the final state is in the terminal region
for k = 1:MPC.Np
    xk = model(xk, u_opt(k));         % update state via nonlinear model
end

if xk.' * MPC.P * xk <= 0.7 
    feas = +1;                  % label as feasible
else
    feas = -1;                  % label as infeasible
end