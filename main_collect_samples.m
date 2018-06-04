clc; clear; close all;

addpath('godlike');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of approximate nonlinear MPC
% Ankush Chakrabarty (chakank@me.com)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialization

nx = 2;             % number of states
nu = 1;             % number of control inputs

Ns = 800;          % number of samples on X

% MPC prediction horizon length
MPC.Np = 15;
MPC.Nu = nu;
MPC.Nx = nx;

% MPC weight parameters
MPC.P = [16.5926, 11.5926; 11.5926, 16.5926];
MPC.Q = diag([2.0, 2.0]);
MPC.R = 0.1;

% Bounds on control actions
MPC.Ulb = -2;
MPC.Uub =  2;

% Bounds on states
MPC.Xlb = -1;
MPC.Xub =  1;

%% Selecting state samples
H = haltonset(nx);
H = scramble(H, 'RR2');
X = MPC.Xlb + (MPC.Xub - MPC.Xlb) * net(H, Ns);     % allocating memory for state samples
clear H

%% Computing optimal control samples

U = zeros(Ns, nu);  % allocating memory for control samples
F = zeros(Ns, 1);   % allocating memory for feasibility information
for k = 1:Ns
    fprintf('\nIteration: %d of %d', k, Ns);
    [U(k), F(k)] = find_optimal_NMPC(X(k,:), MPC);
end


%% Saving samples to save time
save('dataset.mat');