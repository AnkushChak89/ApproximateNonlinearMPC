clc; clear; close all;
load dataset
addpath('godlike');

% Ankush Chakrabarty (chakank@me.com)

%% Detecting the invariant set/feasible region
svm = fitcsvm(X, F, 'Standardize',true,'KernelFunction','rbf',...
                'KernelScale','auto');
            
h = 0.02; % Mesh grid step size
[X1,X2] = meshgrid(-1:h:1, -1:h:1);
[~,score] = predict(svm,[X1(:), X2(:)]);
scoreGrid = reshape(score(:,1), size(X1,1), size(X2,2));

%% Resampling data for the approximate NMPC
Nf = 600;              % number of feasible samples required
Xf = X(F==1,:);         % subset of feasible samples from initial set
Uf = U(F==1,:);         % corresponding control actions
feas_threshold = .10;   % inner approximation of feasible region

% Collecting more samples (Nf) to ensure richness of data for controller
% construction
Xn = [];
Un = [];
while size(Xf, 1) + size(Xn, 1) < Nf
    x = MPC.Xlb + (MPC.Xub - MPC.Xlb) * rand(2,1);  % randomly select state
    if predict(svm, x(:).') > feas_threshold
        [u, ~] = find_optimal_NMPC(x(:).', MPC);
        Xn = [Xn; x(:).'];
        Un = [Un; u(:).'];
        fprintf('\nCollecting data %d of %d', size(Xn,1), Nf - size(Xf,1));
    end
end
Xf = [Xf; Xn];
Uf = [Uf; Un];

%% Plotting
plot(X(F==1,1), X(F==1,2), 'b.',...
        X(F==-1,1), X(F==-1,2), 'r.',...
            Xn(:,1), Xn(:,2), 'y.', 'markersize', 14);
hold on;
contour(X1,X2,scoreGrid,1, 'linewidth', 2);
xlabel('x_1', 'fontsize', 20);
ylabel('x_2', 'fontsize', 20);
h = legend('Feasible', 'Infeasible', 'SVM-based Feasible', 'SVM Boundary');
set(gca, 'fontsize', 20);
set(h, 'fontsize', 20, 'orientation', 'vertical');

%% Storing samples
save('samples_for_ENMPC.mat', 'Xf', 'Xn', 'Uf', 'Un', 'svm');
