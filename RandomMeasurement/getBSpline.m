function shape = getBSpline(x_scale,y_scale,samplingPeriod)

    % Define the shape of the target
    % By changing this parameters, you can obtain differnt shapes
    % Rectangle
    P = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
         0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];

    % Arbitrary Shape
    P = [1.4, 1.3, 1.2, 0.3, 0.2, 0.1, 0.2, 1.3;
         0.1, 1.2, 1.2, 1.3, 1.1, 0.5, 0.4, 0.5];



    

    % The center of the target should be in the middle.
    % This simple operation makes sure that.
    P(1,:) = P(1,:) - mean(P(1,:));
    P(2,:) = P(2,:) - mean(P(2,:));
    
    % The shape information is assumed to be known.
    % Hence, write this to txt.
    % An other file reads the shape during operation.
    writematrix(P,"P.txt")
    

    P(1,:) = x_scale * P(1,:);
    P(2,:) = y_scale * P(2,:);
    
    % Take samples from the shape for visualization
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