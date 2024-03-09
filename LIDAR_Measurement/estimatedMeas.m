function est_Y = estimatedMeas(state,Y)
    number_of_meas = length(Y(1,:));

    % x1 = state(1);
    % x2 = state(2);
    % v  = state(3);
    h  = state(4);
    % w  = state(5);
    x_scale = state(6);
    y_scale = state(7);

    R  = [cos(h) -sin(h)
          sin(h) cos(h)];

    sM = [x_scale 0
          0 y_scale];

    P = readmatrix("P.txt");      
    param_length = length(P);
    
    est_Y = zeros(3,number_of_meas);
    for meas_idx = 1:number_of_meas
        tau = Y(3,meas_idx);
        B = getB2(tau,param_length);
        C = P * B;
        est_Y(1:2,meas_idx) = state(1:2) + R * sM * C;
        est_Y(3,meas_idx) = tau;
    end
end