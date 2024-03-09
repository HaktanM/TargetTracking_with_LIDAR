function [state, covM, est_Y] = measUpdate(state, covM,Y)
    number_of_meas = length(Y(1,:));
    R = eye(number_of_meas*2) * 2.0;

    H = computeH(state, Y);
    S = H * covM * transpose(H) + R;
    K = covM * transpose(H) / S;

    est_Y = estimatedMeas(state, Y);
    y = zeros(number_of_meas*2,1);
    est_y = zeros(number_of_meas*2,1);
    for meas_idx = 1:number_of_meas
        y(2*meas_idx-1:2*meas_idx) = Y(1:2,meas_idx);
        est_y(2*meas_idx-1:2*meas_idx) = est_Y(1:2,meas_idx);
    end
    state = state + K * (y - est_y);
    covM = covM - K * H * covM;
end