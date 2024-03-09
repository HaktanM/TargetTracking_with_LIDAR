function state = MotionModel(prev_state, dT)
    x1 = prev_state(1);
    x2 = prev_state(2);
    v  = prev_state(3);
    h  = prev_state(4);
    w  = prev_state(5);
    
    state = -ones(5,1);
    if w < 0.0000000001
        state(1) = x1 + v*dT * cos(h + w*dT/2);
        state(2) = x2 + v*dT * sin(h + w*dT/2);
    else
        state(1) = x1 + 2*v/w * sin(w*dT/2) * cos(h + w*dT/2);
        state(2) = x2 + 2*v/w * sin(w*dT/2) * sin(h + w*dT/2);
    end

    state(3) = v + (rand-0.5)*10*dT;
    state(4) = h + w*dT;
    state(5) = w + (rand-0.5)*10/180*pi*dT;
end