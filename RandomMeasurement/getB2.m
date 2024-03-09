function B=getB2(tau,param_length)
    M = [0.5, -1, 0.5;
         -1 ,  1, 0.5;
         0.5,  0,   0];   
    first_active = floor(tau+1);
    tau_active = tau - first_active + 1;
    tau_vec = [tau_active^2; tau_active; 1];
    B = zeros(param_length,1);
    newM = M*tau_vec;
    for i = 0:2
        index = first_active + i;
        while index > param_length 
            index = index - param_length;
        end
        B(index,1) = newM(i+1);
    end
end