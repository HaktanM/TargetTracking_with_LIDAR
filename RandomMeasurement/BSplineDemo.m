close all
clear
clc

%P = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0;
%     0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0 ,0.0];

%P(1,:) = P(1,:) - mean(P(1,:));
%P(2,:) = P(2,:) - mean(P(2,:));

P = readmatrix("P.txt"); 
param_length = length(P);
BasesFunctions=zeros(10*param_length+1,param_length);
x = [];
y = [];

idx = 1;
for i = 0:0.1:param_length
    tau = i;
    B = getB2(tau,param_length);
    C = P * B;
    x = [x, C(1)];
    y = [y, C(2)];

    BasesFunctions(idx,:) = B;
    idx = idx + 1;
end
tau = 0:0.1:param_length;

f1 = figure('Position', [200 150 500 400]);
f2 = figure('Position', [800 50 500 700]);

figure(2)
subplot(2,1,1)
plot(tau,x,LineWidth=2,Color="blue")
hold on
for base_idx = 1:8
    plot(tau,BasesFunctions(:,base_idx)*P(1,base_idx),"--",LineWidth=1)
end

subplot(2,1,2)
plot(tau,y,LineWidth=2,Color="blue")
hold on
for base_idx = 1:8
    plot(tau,BasesFunctions(:,base_idx)*P(2,base_idx),"--",LineWidth=1)
end


for i = 1:81
    figure(1)
    scatter(x(1:i),y(1:i),25,'red','filled')
    xlim([-0.6, 0.6])
    ylim([-0.6, 0.6])
    
    grid on
    set(gca,'Xtick',-0.6:0.1:0.6)
    set(gca,'Ytick',-0.6:0.1:0.6)
    
    xlabel('x (meter)','Interpreter','latex')
    ylabel('y (meter)','Interpreter','latex')
    title('B-Splines Representation of a Rectangle','Interpreter','latex')

    figure(2)
    subplot(2,1,1)
    scatter(tau(i),x(i),25,'red','filled')
    ylabel('$x(\tau)$','Interpreter','latex')
    grid on

    subplot(2,1,2)
    scatter(tau(i),y(i),25,'red','filled')
    xlabel('$\tau$','Interpreter','latex')
    ylabel('$y(\tau)$','Interpreter','latex')
    grid on
    pause(0.5)
end