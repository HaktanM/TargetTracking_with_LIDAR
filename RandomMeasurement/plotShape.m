P = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
     0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
    
P(1,:) = P(1,:) - mean(P(1,:));
P(2,:) = P(2,:) - mean(P(2,:));

x = [];
y = [];
param_length = length(P);
sampling_period = 0.01;
for i = 0:sampling_period:param_length
    tau = i;
    B = getB2(tau,param_length);
    C = P * B;
    x = [x, C(1)];
    y = [y, C(2)];
end
tau = 0:sampling_period:param_length;

f = figure('Position', [200 200 500 400]);
scatter(x,y,[],tau,'filled')
colorbar
xlim([-0.5, 0.5])
ylim([-0.5, 0.5])

grid on
set(gca,'Xtick',-0.6:0.1:0.6)
set(gca,'Ytick',-0.6:0.1:0.6)

xlim([-0.6 0.6])
ylim([-0.6 0.6])
xlabel('x (meter)','Interpreter','latex')
ylabel('y (meter)','Interpreter','latex')
title('B-Splines Representation of an Arbitrary Shape','Interpreter','latex')

% saveas(f,"arbitraryShape8Basis.png")