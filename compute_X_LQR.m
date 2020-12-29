% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    try
        load('X_LQR');
        return
    catch err
        disp('X_LQR not found, calculating...')
    end
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.  
    [F_inf, ~, ~] = dlqr(param.A, param.B, param.Q, param.R, 0);
    K = -F_inf;
    
    systemLQR = LTISystem('A', param.A - param.B * K);
    systemLQR.x.min = param.Xcons(:, 1);
    systemLQR.x.max = param.Xcons(:, 2);
    
    u_min = param.Pcons(:, 1);
    u_max = param.Pcons(:, 2);
    
    Xp = Polyhedron('A', [eye(3); -eye(3); K; -K], ...
        'b', [systemLQR.x.max;-systemLQR.x.min; u_max;-u_min]);
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    
    InvSetLQR = systemLQR.invariantSet();
    
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
    save('X_LQR', 'A_x', 'b_x');
end

