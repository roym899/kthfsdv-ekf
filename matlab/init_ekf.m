function [x, P] = init_ekf(ekf)
    if ekf.settings.two_dimensional
        x = [0,0,0,0]; % x,y,theta,v
        P = diag([1,1,1,1]);
    end
end