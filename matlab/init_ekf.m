function [x0] = init_ekf(ekf)
    if ekf.settings.two_dimensional
        x0 = [0,0,0,0]; % x,y,theta,v
    end
end