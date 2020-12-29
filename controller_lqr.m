% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% Retrieve controller parameters
B = param.B;
A = param.A;
F = param.F;
T_sp = param.T_sp;
p_sp = param.p_sp;
Q = param.Q;
R = param.R;

% compute control action
A_k = A - B*F;
steps = 2; % simulate infinite horizon
x = T-T_sp; % initialise state x
x_0 = x; %save initial state
cost = 0; % initialise cost
u = -F*x; % compute control input
for i = 1:steps
    cost = cost + x.'*Q*x + u.'*R*u; % compute cost
    x = A_k * x; % compute next state
%     if i == 1
%         disp(['||T_sp - T(1)|| = ', num2str(norm(x))])
%         disp(['0.2||x_0|| = ', num2str(0.2*norm(x_0,2))])
%     end
end
p = u + p_sp; % retrieve power required
param.cost = cost;

end

function param = init()
param = compute_controller_base_parameters;
% Compute the infinte horizon F and P
[F_inf, P_inf, ~] = dlqr(param.A, param.B, param.Q, param.R, 0);
% add additional parameters if necessary, e.g.
param.F = F_inf;
param.P = P_inf;

end
