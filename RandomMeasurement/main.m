close all
clear
clc

% Initial location and orientation of the target
center = [20 10];     % Location
heading = 0;        % Orientation

% Initialize the ACTUAL State
state = zeros(7,1);
state(1:2) = center;
state(4) = heading;
state(3) = 2;
state(5) = 0.3;
state(6) = 3;
state(7) = 2;

% Initialize the ESTIMATED state
estim_state = zeros(7,1);
estim_state(1:7) = state(1:7);
CovM = eye(7) * 1;

% Add some corrupted information
estim_state(1) = estim_state(1) + 1; 
estim_state(2) = estim_state(2) + 1.5;
estim_state(4) = estim_state(4) + 2 / 180 * pi;
estim_state(6) = estim_state(6) * 1.5;
estim_state(7) = estim_state(7) * 0.75;

% Set this to 0 to cancel the measurement update
takeMeas = 1;
% By canceling the measurement update,
% you can visualize the amount of the noise in the overall system

% The time interval between estimations
dT = 0.1;

% Initialize the target
% I have defined an object holding 
% the shape and pose information of the target 
target_shape = getBSpline(state(6),state(7),0.1); % For visualization, the shape is discretized
target_obj = target(target_shape,center,heading);

shape_length = length(target_shape(:,1));
tau = 0:0.1:8;

% Initialize the estimation
estim_shape = getBSpline(estim_state(6),estim_state(7),0.1);
estim_obj = target(estim_shape,estim_state(1:2),estim_state(4));

% I will save the estimated state
printer_obj = loader("Estimation.txt");


f = figure;
f.Position = [300 50 750 700];
for i = 1:1000
    
    % Propagate the ACTUAL state. This is my simulator
    state(1:5) = MotionModel(state(1:5), dT);

    % Modify the ACTUAL taget pose. This is my simulator
    target_obj.modifyPose([state(1),state(2)],state(4))
    boundary = target_obj.getBoundary();


    % Get measurements
    number_of_meas = floor(rand*5+1);
    Y = zeros(3,number_of_meas);
    for meas_idx = 1:number_of_meas
        Y(:,meas_idx) = getMeasurement(boundary,tau);
    end

    
    % ESTIMATION STARTS HERE
    valid_meas = [];
    meas_status = zeros(number_of_meas,1);
    for meas_idx = 1:number_of_meas
        % first we need to estimate the abstract parameter tau
        [tau_estimation, est_statu] = estimateTau2(Y(:,meas_idx),state);
        if est_statu
            meas = [Y(1:2,meas_idx);tau_estimation];
            valid_meas = [valid_meas meas];
            meas_status(meas_idx) = 1;
        end
    end
    if isempty( valid_meas )
        continue
    else
        number_of_vld_meas = length(valid_meas(1,:));
    end
    

    old_scale = [estim_state(6) estim_state(7)];
    % TIME UPDATE
    [estim_state(1:5), CovM(1:5,1:5)] = timeUpdate(estim_state(1:5), CovM(1:5,1:5), dT);
    
    % MEASUREMENT UPDATE
    if takeMeas
        [estim_state, CovM, est_Y] = measUpdate(estim_state, CovM, valid_meas);
        scalaM = [estim_state(6)/old_scale(1) 0
                  0 estim_state(7)/old_scale(2)] ;
        estim_obj.resizeShape(scalaM)
    end

    % ESTIMATION ENDED FOR THIS EPOCH
    % THE REST IS VISUALIZATION

    % Append our txt file
    printer_obj.append_state(estim_state)

    % Modify the ESTIMATED pose
    estim_obj.modifyPose([estim_state(1),estim_state(2)],estim_state(4))

    % Where is the boundary of the taget (for visualization)
    est_boundary = estim_obj.getBoundary();

    
    % Start Plotting Everything
    scatter(boundary(:,1),boundary(:,2),10,tau,'filled')
    colorbar
    hold on
    plot(est_boundary(:,1),est_boundary(:,2),LineWidth=3)
    
    scatter(state(1), state(2), 100, "k", "filled")
    plot([state(1) state(1)+2*cos(state(4))], [state(2) state(2)+2*sin(state(4))], "k")
    
    scatter(estim_state(1), estim_state(2), 100, "blue", "filled")
    plot([estim_state(1) estim_state(1)+2*cos(estim_state(4))], [estim_state(2) estim_state(2)+2*sin(estim_state(4))], "blue")
    
    
    if takeMeas
        for meas_idx = 1:number_of_vld_meas
            y = est_Y(:,meas_idx);
            scatter(y(1),y(2),100,"red","filled")
            y = Y(:,meas_idx);
            scatter(y(1),y(2),100,"green","filled")
        end
    end

    h1 = plot_gaussian_ellipsoid([estim_state(1) estim_state(2)], CovM(1:2,1:2));
    set(h1,'color','b');
    
    xlabel('x (meter)','Interpreter','latex')
    ylabel('y (meter)','Interpreter','latex')
    title('Target and Estimation in Global Scene','Interpreter','latex')
    grid on
    

    xlim([0 40])
    ylim([0 40])

    saveas(gcf,"out3\"+string(i)+".png")
    pause(0.01)
    hold off
end