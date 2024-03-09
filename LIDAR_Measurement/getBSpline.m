function shape = getBSpline(x_scale,y_scale,samplingPeriod)
    
    % Rectangle
    P = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
         0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
    
    
    
    P(1,:) = P(1,:) - mean(P(1,:));
    P(2,:) = P(2,:) - mean(P(2,:));
    
    writematrix(P,"P.txt")

    P(1,:) = x_scale * P(1,:);
    P(2,:) = y_scale * P(2,:);

    x = [];
    y = [];
    param_length = length(P);
    for i = 0:samplingPeriod:param_length
        tau = i;
        B = getB2(tau,param_length);
        C = P * B;
        x = [x, C(1)];
        y = [y, C(2)];
    end
    shape = zeros(length(x), 2);
    shape(:,1) = x;
    shape(:,2) = y;
end