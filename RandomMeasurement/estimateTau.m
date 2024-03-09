function tau = estimateTau(meas, state)
    % x1 = state(1);
    % x2 = state(2);
    % v  = state(3);
    heading  = state(4);
    % w  = state(5);
    % x_scale = state(6);
    % y_scale = state(7);

    loc = state(1:2);
    R = zeros(2,2);
    R(1,1) = cos(heading);
    R(1,2) = -sin(heading);
    R(2,1) = sin(heading);
    R(2,2) = cos(heading);
   
    C = transpose(R) * (meas(1:2) - loc);
    bound_x = C(1);
    bound_y = C(2);
    meas_angle = atan(bound_y/bound_x);
    if bound_x<0
        meas_angle = meas_angle + pi;
    end
    while meas_angle<0
        meas_angle = meas_angle + 2*pi;
    end
    tau_interval = ceil(meas_angle/(2*pi)*8);

    tau=tau_interval;
end