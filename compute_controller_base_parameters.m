function param = compute_controller_base_parameters
clear;    
% load truck parameters
    load('system/parameters_truck');
    m1 = truck.m1;
    m2 = truck.m2;
    m3 = truck.m3;
    To = truck.To;
    w = truck.w;
    a12 = truck.a12;
    a1o = truck.a1o;
    a23 = truck.a23;
    a2o = truck.a2o;
    a3o = truck.a3o;
    
    % (2) discretization
    Ts = 60;
    % Continuous matrices
    A_c = [-(a12+a1o)/m1, a12/m1, 0; ...
        a12/m2, -(a12+a23+a2o)/m2, a23/m2; ...
        0, a23/m3, -(a23+a3o)/m3];
    B_c = [1/m1, 0; 0, 1/m2; 0, 0];
    B_c_d = [1/m1, 0, 0; 0, 1/m2, 0; 0, 0, 1/m3];
    d_c = [a1o; a2o; a3o]*To + w;
    
    % discretisation euler
    A = (eye(3) + Ts*A_c);
    B = Ts*B_c;
    
    % (3) set point computation
    T1_ref = -21;
    T2_ref = 0.3;
    T3_ref = (w(3, 1) + a3o*To + a23*T2_ref)/(a23+a3o);
    T_sp = [T1_ref; T2_ref; T3_ref];
    p_sp = [-w(1, 1) - a12*(T2_ref - T1_ref) - a1o*(To - T1_ref); ...
        -w(2, 1) - a12*(T1_ref - T2_ref) - a23*(T3_ref - T2_ref) - a2o*(To - T2_ref)];
    
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = [Pcons(:, 1) - p_sp, Pcons(:, 2) - p_sp];
    Xcons = [Tcons(:, 1) - T_sp, Tcons(:, 2) - T_sp];
    
    % (5) LQR cost function
    Q = [500, 0, 0; 0, 500, 0; 0, 0, 0];
    R = 1e-2*eye(2);
    % For T0_2
%     Q = [100, 0, 0; 0, 2000, 0; 0, 0, 0];
%     R = 1e-3*eye(2);
    % For T0_1
%     Q = diag([500, 500, 0]);
%     R = 1e0*eye(2);
    % For T0_2
%     Q = diag([1, 50000, 0]);
%     R = diag([0.08, 0.5]);
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

