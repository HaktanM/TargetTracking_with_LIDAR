function H = computeH(state,Y)
    number_of_meas = length(Y(1,:));
    H = zeros(number_of_meas*2,length(state));

    % x1 = state(1);
    % x2 = state(2);
    % v  = state(3);
    h  = state(4);
    % w  = state(5);
    x_scale = state(6);
    y_scale = state(7);

    R  = [cos(h) -sin(h)
          sin(h) cos(h)];

    Rp = [-sin(h) -cos(h)
           cos(h) -sin(h)];

    sM = [x_scale 0
          0 y_scale];

    P = readmatrix("P.txt");    
    param_length = length(P);

    for meas_idx = 1:number_of_meas
        y = Y;
        H(2*meas_idx-1:2*meas_idx,1:2) = eye(2); % del h / del p
        
        % del h / del heading
        tau = y(3);
        B = getB2(tau,param_length);
        C = P * B;
        H(2*meas_idx-1:2*meas_idx,4) = Rp * sM * C;

        % del h / del scale
        H(2*meas_idx-1:2*meas_idx,6:7) = R * [C(1) 0 
                                          0 C(2)];
    end
end