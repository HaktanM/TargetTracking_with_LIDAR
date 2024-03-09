function [tau_est, est_statu] = estimateTau2(meas, state)
    % x1 = state(1);
    % x2 = state(2);
    % v  = state(3);
    h  = state(4);
    % w  = state(5);
    x_scale = state(6);
    y_scale = state(7);

    loc = state(1:2);
    
    R  = [cos(h) -sin(h)
          sin(h) cos(h)];

    sM = [x_scale 0
          0 y_scale];

    P = readmatrix("P.txt");      
    param_length = length(P);

    M = [0.5, -1, 0.5;
         -1 ,  1, 0.5;
         0.5,  0,   0];
    
    C_from_meas = transpose(R) * (meas(1:2) - loc);
    
    mid_points = zeros(2,param_length);
    error_vec = zeros(1,param_length);
    for tau_idx = 1:param_length
        mid_tau = tau_idx-0.5;
        B = getB2(mid_tau,param_length);
        C_mid = sM * P * B;
        mid_points(:,tau_idx) = C_mid;
        error = norm(C_from_meas - C_mid);
        error_vec(tau_idx) = error;
    end

    % Find the minimum value in the array
    minErr = min(error_vec);
    
    % Find the index of the minimum value
    minErrIndex = find(error_vec == minErr, 1);

    P_2by3 = zeros(2,3);
    for idx = 0:2
        tau_idx = minErrIndex + idx;
        if tau_idx > 8
            tau_idx = tau_idx - 8;
        end
        P_2by3(:,idx+1) = P(:,tau_idx);
    end

    sPM = sM * P_2by3 * M;
    a = sPM(:,1);
    a(3) = 0;
    b = sPM(:,2);
    b(3) = 0;
    c = sPM(:,3);
    c(3) = 0;

    y_h = [C_from_meas;0];

    u_a = cross(a,y_h);
    u_b = cross(b,y_h);
    u_c = cross(c,y_h);

    tau_frac_roots = roots([u_a(3) u_b(3) u_c(3)]);

    % Check if both roots are real
    if ~isreal(tau_frac_roots(1)) && ~isreal(tau_frac_roots(2))
        tau_frac = 0;
        est_statu = false;
    elseif tau_frac_roots(1)*tau_frac_roots(2)<0
        tau_frac = max(tau_frac_roots);
        est_statu = true;
    else
        tau_frac = min(tau_frac_roots);
        est_statu = true;
    end
    tau_est = minErrIndex - 1 + tau_frac;
end