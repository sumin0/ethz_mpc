% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
[u_mpc,errorcode] = yalmip_optimizer{T - param.T_sp};
if (errorcode ~= 0)
      warning('MPC infeasible');
end
% u_mpc
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip here, e.g.
% https://yalmip.github.io/example/standardmpc/

N = 30;
nx = size(param.A,1);
nu = size(param.B,2);
X = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1),'full'); % N (3x1) x's
U = sdpvar(repmat(nu,1,N),repmat(1,1,N),'full');

A = param.A;
B = param.B;
Q = param.Q;
R = param.R;

objective = 0;
% constraints = [];
for k = 1:N
    if k == 1
        constraints = [X{k+1} == A*X{k} + B*U{k}, ...
            param.Pcons(:, 1) <= U{k} <= param.Pcons(:, 2)];
    else
        constraints = [constraints, ...
            X{k+1} == A*X{k} + B*U{k}, ...
            param.Xcons(:, 1) <= X{k} <= param.Xcons(:, 2), ...
            param.Pcons(:, 1) <= U{k} <= param.Pcons(:, 2)];
    end
    objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
end

[~, P, ~] = dlqr(param.A, param.B, param.Q, param.R, 0);
% [A_x, b_x] = compute_X_LQR;

% terminal set
% constraints = [constraints, A_x * X{end} <= b_x];
% terminal objective
objective = objective + X{end}'*P*X{end};
% parameter for initial condition
x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

ops = sdpsettings('verbose',0,'solver','quadprog');
fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops, x0, U{1});
end