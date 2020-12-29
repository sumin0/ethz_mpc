
% 
clc
clear variables
parameters = compute_controller_base_parameters;
load('system/parameters_truck');
load('system/parameters_scenarios');
T0_1 = parameters.T_sp + [3; 1; 0];
T0_2 = parameters.T_sp + [-1; -0.3; -4.5];
T0_3 = [12; 12; 12];
% lambdas
controller_lqr_lambda = @(T)controller_lqr(T);
controller_mpc_1_lambda = @(T)controller_mpc_1(T);
controller_mpc_2_lambda = @(T)controller_mpc_2(T);
controller_mpc_3_lambda = @(T)controller_mpc_3(T);
controller_mpc_4_lambda = @(T)controller_mpc_4(T);

%clear param
%controller_mpc_4_lambda(T0_2)
%figure(2)
simulate_truck(T0_2,controller_mpc_3_lambda, scen1);

%set figure properties
axises = get(gcf, 'children');
for i= 1:length(axises)
   try
       lines = get(axises(i), 'children');
       for j = 1:length(lines)
           set(lines(j), 'LineWidth', 1);
       end
   catch e
       
   end
end

% %Plot costs
% load('J_MPC1s_T1.mat')
% load('J_MPC2s_T1.mat')
% plot(J_MPC1s, 'DisplayName','J_MPC1', 'linewidth', 1)
% hold on
% plot(J_MPC2s, 'DisplayName','J_MPC2', 'linewidth', 1)
% legend('J_{MPC1}','J_{MPC2}')
% title('Q13: Cost comparison for T_{init}^{(1)}')

%polytope plot
%[G, h] = compute_X_LQR;
%figure(2)
%plotregion({-G(1:64,:), -G(1:61,:)},{-h(1:64,:),-h(1:61,:)},{[],[]},{[],[]},{[0.7,0.2,0.3],[0.7,0.2,0.3]});
%plotregion(-G(1:61,:),-h(1:end,:),{[],[]},{[],[]},[0.7,0.2,0.3]);

%  hold on
%  plot3(T0_1(1),T0_1(2),T0_1(3),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
%  plot3(0,0,0,'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
%  hold on
%  plot3(T0_2(1),T0_2(2),T0_2(3),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
% 
