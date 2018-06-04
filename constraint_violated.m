function q = constraint_violated(x, MPC)

% Check state constraints
% Ankush Chakrabarty (chakank@me.com)
q = 1;
for k = 1:MPC.Nx
    if x(k) > MPC.Xub || x(k) < MPC.Xlb
        q = -1;
    end
end