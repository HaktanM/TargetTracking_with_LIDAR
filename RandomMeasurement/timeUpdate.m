function [state, CovM] = timeUpdate(prev_state, prevCov, dT)
    x1 = prev_state(1);
    x2 = prev_state(2);
    v  = prev_state(3);
    h  = prev_state(4);
    w  = prev_state(5);

    G = zeros(5,2);
    G(3,1) = 5;
    G(5,2) = 2;

    Q = zeros(2,2);
    Q(1,1) = 2.0;
    Q(2,2) = 0.08;

    Q = Q * dT;
    
    % Mean Update
    state = -ones(5,1);
    if w < 0.0000000001
        state(1) = x1 + v*dT * cos(h + w*dT/2);
        state(2) = x2 + v*dT * sin(h + w*dT/2);
    else
        state(1) = x1 + 2*v/w * sin(w*dT/2) * cos(h + w*dT/2);
        state(2) = x2 + 2*v/w * sin(w*dT/2) * sin(h + w*dT/2);
    end
    state(3) = v;
    state(4) = h + w*dT;
    state(5) = w;

    % Cov Update
    F = eye(5);
    F(4,5) = dT;
    if w < 0.0000000001
        F(1,3) = 2/w * sin(w*dT/2) * cos(h + w*dT/2);
        F(1,4) = - 2*v/w * sin(w*dT/2) * sin(h + w*dT/2);
        F(1,5) = - 2*v/(w*w) * sin(w*dT/2) * cos(h + w*dT/2) + (v/w) * dT * (cos(w*dT/2) * cos(h + w*dT/2) - sin(w*dT/2) * sin(h + w*dT/2));

        F(2,3) = 2/w * sin(w*dT/2) * sin(h + w*dT/2);
        F(2,4) = 2*v/w * sin(w*dT/2) * cos(h + w*dT/2);
        F(2,5) = - 2*v/(w*w) * sin(w*dT/2) * sin(h + w*dT/2) + (v/w) * dT * (cos(w*dT/2) * sin(h + w*dT/2) + sin(w*dT/2) * cos(h + w*dT/2));
    else
        F(1,3) = dT * cos(h);
        F(1,4) = -v*dT*sin(h);

        F(2,3) = dT * sin(h);
        F(2,4) = v * dT * cos(h);
    end

    CovM = F * prevCov * transpose(F) + G * Q * transpose(G);
end