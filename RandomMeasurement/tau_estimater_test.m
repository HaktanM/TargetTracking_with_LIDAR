close all
clear 
clc

% Initial location and orientation of the target
center = [20 10];     % Location
heading = 0;        % Orientation
state = zeros(7,1);
state(1:2) = center;
state(4) = heading;

state(3) = 2;
state(5) = 0.3;

state(6) = 1.8;
state(7) = 2.2;

takeMeas = 1;

estim_state = zeros(7,1);
estim_state(1:7) = state(1:7);
CovM = eye(7) * 1;

% Add some corrupted information
estim_state(1) = estim_state(1); 
estim_state(2) = estim_state(2);
estim_state(4) = estim_state(4);
estim_state(6) = estim_state(6);
estim_state(7) = estim_state(7);

% Initialize the target
sampling_period = 0.01; % Not varified
target_shape = getBSpline(state(6),state(7),sampling_period);
target_obj = target(target_shape,center,heading);

shape_length = length(target_shape(:,1));
tau = 0:sampling_period:8;

% Initialize the estimation
estim_shape = getBSpline(estim_state(6),estim_state(7),sampling_period);
estim_obj = target(estim_shape,estim_state(1:2),estim_state(4));

f = figure;
f.Position = [800 50 750 700];
for i = 1:100000
    state(1:5) = MotionModel(state(1:5), 0.1);

    % Modify the taget pose
    target_obj.modifyPose([state(1),state(2)],state(4))
    boundary = target_obj.getBoundary();

    %Get the target shape expressed in local coordinate frame
    target_in_l = target_obj.target_shape;


    % Get measurement
    number_of_meas = floor(rand*6+1);
    Y = zeros(3,number_of_meas);
    for meas_idx = 1:number_of_meas
        Y(:,meas_idx) = getMeasurement(boundary,tau);
    end

    [tau_est, mid_points, C_from_meas] = estimateTau2(Y(:,1),state);
    
    tau_est;
    tau_act = Y(3,1);

    tau_err = tau_est - tau_act;

    if abs(tau_err)>0.0001
        tau_est
        tau_act = Y(3,1)
        tau_err
        break
    end
    
    
    % Plots
    scatter(target_in_l(:,1),target_in_l(:,2),10,tau,'filled')
    colorbar
    hold on
    
    for tau_idx=1:8
        scatter(mid_points(1,tau_idx),mid_points(2,tau_idx),100, "red",'filled')
        scatter(C_from_meas(1),C_from_meas(2),100, "black",'filled')
    end
    
    xlabel('x (meter)','Interpreter','latex')
    ylabel('y (meter)','Interpreter','latex')
    title('Local Coordinate Frame Visualization','Interpreter','latex')
    grid on
    

    xlim([-2 2])
    ylim([-2 2])
    pause(0.001)
    hold off
end