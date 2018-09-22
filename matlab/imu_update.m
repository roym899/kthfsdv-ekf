function [x_hat, P_hat] = imu_update(ekf, dt, x, P, Q, u)
    % IMU is used as the prediction step inside the EKF
    if ekf.settings.two_dimensional
        x_hat = [x(1)+x(4)*cos(x(3))*dt x(2)+x(4)*sin(x(3))*dt x(3)+u(1)*dt x(4)+u(2)*dt];
        grad_f_x = [1 0 -x(4)*sin(x(3))*dt cos(x(3))*dt;
                    0 1 x(4)*cos(x(3))*dt sin(x(3))*dt;
                    0 0 1 0;
                    0 0 0 1];
        grad_f_v = [0 0;
                    0 0;
                    -dt 0;
                    0 dt];
        P_hat = grad_f_x*P*grad_f_x'+grad_f_v*Q*grad_f_v';
    end
end

