close all
clear
clc
% rng(255) % 25 35 105 255
% Initial location and orientation of the target
center = [20 10];     % Location
heading = 0;        % Orientation
state = zeros(7,1);
state(1:2) = center;
state(4) = heading;

state(3) = 2;
state(5) = 0.3;

state(6) = 3;
state(7) = 2;

takeMeas = 1;

estim_state = zeros(7,1);
estim_state(1:7) = state(1:7);
CovM = eye(7) * 1;

% Add some corrupted information
estim_state(1) = estim_state(1) ;%+ 1; 
estim_state(2) = estim_state(2) ;%+ 1.5;
estim_state(4) = estim_state(4) ;%+ 2 / 180 * pi;
estim_state(6) = estim_state(6) ;%* 1.5;
estim_state(7) = estim_state(7) ;%* 0.75;

% Initialize the lidar sensor
rbsensor = rangeSensor;
rbsensor.Range = [0 40];
truePose = [0 0 0];   % Location and orientation of the lidar

% This part is used to draw a triangle representing the lidar, not important
% Just ignore here
triangle = nsidedpoly(3, 'Center', truePose(1:2), 'SideLength', 2);
lidar_corners = zeros(3,2);
lidar_corners(:,1) = [0 0 2];
lidar_corners(:,2) = [-0.8 0.8 0];
R = zeros(2,2);
R(1,1) = cos(truePose(3));
R(1,2) = -sin(truePose(3));
R(2,1) = sin(truePose(3));
R(2,2) = cos(truePose(3));
lidar_corners = lidar_corners * transpose(R);
lidar_x = lidar_corners(:,1) + [truePose(1) truePose(1) truePose(1)];
lidar_y = lidar_corners(:,2) + [truePose(2) truePose(2) truePose(2)];

% Initialize the target
target_shape = getBSpline(state(6),state(7),0.1);
target_obj = target(target_shape,center,heading);

shape_length = length(target_shape(:,1));
tau = 0:0.1:8;

% Initialize the estimation
estim_shape = getBSpline(estim_state(6),estim_state(7),0.1);
estim_obj = target(estim_shape,estim_state(1:2),estim_state(4));

f = figure;
f.Position = [300 50 750 700];
for i = 1:1000
    state(1:5) = MotionModel(state(1:5), 0.1);

    % Modify the taget pose
    target_obj.modifyPose([state(1),state(2)],state(4))
    boundary = target_obj.getBoundary();

    % First create an empt matrix,
    % Then mark the boundaires of the target
    M = zeros(4000,4000);
    for bound_idx=1:shape_length  % For each boundry point on the target
        % Convert the boundary information to matrix index
        % Example:
        % Boundary point is located on (3meter, 4meter)
        % This corresponds to (x_loc,y_loc) index in the matrix
        x_loc = round(boundary(bound_idx,1) * 100) ; 
        y_loc = 4000 - round(boundary(bound_idx,2) * 100) ;

        % Sadece boundary point değil, çevresindeki pikseller de 
        % dolu olarak işaretle
        M(y_loc-10:y_loc+10,x_loc-10:x_loc+10) = ones(21,21);
    end

    % Create the binary map for the lidar
    trueMap = binaryOccupancyMap(M,100);
    
    % Take the measurement
    [ranges, angles] = rbsensor(truePose, trueMap);
    scan = lidarScan(ranges, angles);
    lidar_meas = scan.Cartesian;
    vld_lidar_meas = [];
    for lidar_meas_idx = 1:scan.Count
        if ~isnan(lidar_meas(lidar_meas_idx,1))
            vld_lidar_meas = [vld_lidar_meas transpose(lidar_meas(lidar_meas_idx,:))];
        end
    end

    if isempty( vld_lidar_meas )
        continue
    else
        number_of_meas = length(vld_lidar_meas(1,:));
    end

    
    % Estimation
    valid_meas = [];
    meas_status = zeros(number_of_meas,1);
    for meas_idx = 1:number_of_meas
        [tau_estimation, est_statu] = estimateTau2(vld_lidar_meas(:,meas_idx),state);
        if est_statu
            meas = [vld_lidar_meas(1:2,meas_idx);tau_estimation];
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
    [estim_state(1:5), CovM(1:5,1:5)] = timeUpdate(estim_state(1:5), CovM(1:5,1:5), 0.1);
    if takeMeas
        [estim_state, CovM, est_Y] = measUpdate(estim_state, CovM, valid_meas);
        scalaM = [estim_state(6)/old_scale(1) 0
                  0 estim_state(7)/old_scale(2)] ;
        estim_obj.resizeShape(scalaM)
    end

    % Modify the pose
    estim_obj.modifyPose([estim_state(1),estim_state(2)],estim_state(4))

    % Where is the boundary of the taget
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
    
    
    scatter(est_Y(1,:),est_Y(2,:),100,"red","filled")
    scatter(vld_lidar_meas(1,:),vld_lidar_meas(2,:),100,"green","filled")

    h1 = plot_gaussian_ellipsoid([estim_state(1) estim_state(2)], CovM(1:2,1:2));
    set(h1,'color','b');
    
    % Illustrate the lidar
    patch(lidar_x, lidar_y, 'r');
    
    xlabel('x (meter)','Interpreter','latex')
    ylabel('y (meter)','Interpreter','latex')
    title('Target and Estimation in Global Scene','Interpreter','latex')
    grid on
    

    xlim([-1 40])
    ylim([-1 40])
    
    saveas(gcf,"out2\"+string(i)+".png")
    pause(0.01)
    hold off
end