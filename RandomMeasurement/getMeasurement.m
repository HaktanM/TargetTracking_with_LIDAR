function y = getMeasurement(boundary, tau)
    idx = floor(rand * length(boundary)+1);
    y = [boundary(idx,1) + (rand-0.5)*0.5
         boundary(idx,2) + (rand-0.5)*0.5
         tau(idx)];
end