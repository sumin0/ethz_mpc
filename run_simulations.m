% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr controller_mpc_1; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
param = compute_controller_base_parameters;
x_0_1 = [3; 1; 0]; % initial state (Q. 5)
x_0_2 = [-1; -0.3; -4.5]; % initial state (Q. 7)
T0_1 = param.T_sp + x_0_1; % initial temperature (Q. 5)
T0_2 = param.T_sp + x_0_2; % initial temperature (Q. 7)
% [T_lqr, p_lqr] = simulate_truck(T0_1, @controller_lqr, scen1);
[T_mpc1, p_mpc1] = simulate_truck(T0_2, @controller_mpc_1, scen1);

% need to include the FORCES package
% addpath(genpath("..."))
% [~, ~, t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
% [~, ~, t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);

%%
% param.A
% eig(param.A)
% For Q5
% if (norm(x_0_1)*0.2 - norm(param.T_sp-T_lqr(:, 30))) > 0
%     disp('Q5 condition satisfied!')
% else
%     disp('Failed - Q5 condition not satisfied')
% end
