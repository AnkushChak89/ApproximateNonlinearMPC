clc; clear; close all;
rng('default');

% Ankush Chakrabarty (chakank@me.com)

%% Control surface regression

% Loading controller samples (within feasible region)
load('samples_for_ENMPC.mat', 'Xf', 'Uf', 'svm');

S = @(x,y) [1, x, y, x*y, (2*x^2-1)*y, (2*y^2-1)*x,...
            (4*x^3 - 3*x), (4*y^3 - 3*y)];

for k = 1:size(Xf,1)
    G(k,:) = S(Xf(k,1), Xf(k,2));
end

c = G\Uf;
fprintf('\nDisplaying coefficients of the controller...\n');
disp(table(c));

fprintf('\nDisplaying basis functions...\n');
disp(S);

%% Testing closed-loop control
T = 10;      % time of simulation
load('dataset.mat', 'MPC');

% Choosing an initial state that is feasible
start_feasible = 0;
while start_feasible == 0
    x0 = MPC.Xlb + rand(2,1)*(MPC.Xub - MPC.Xlb);
    start_feasible = predict(svm, x0(:).') > 0.1;
end

% Running Simulink file
fprintf('\nRunning Simulink file...');
figure(1);
sim('simulation_ENMPC.slx');
tx = xhist.Time.';
x = xhist.Data.';

%% Plotting
subplot(2,1,1);
plot(tx, x([1,2],:).', 'linewidth', 2);
h = legend('x_1', 'x_2');
set(h, 'orientation', 'horizontal');
set(gca, 'fontsize', 20);
xlabel('t');
ylabel('x(t)');
hold on
subplot(2,1,2);
tu = uhist.Time;
u = squeeze(uhist.Data);
plot(tu, u, 'linewidth', 2);
set(h, 'orientation', 'horizontal');
set(gca, 'fontsize', 20);
xlabel('t');
ylabel('u(t)');
hold on