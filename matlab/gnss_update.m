function [x_hat, P_hat] = gnss_update(ekf, x, P, R, z)
    % partial measurement update
    h = [x(1) x(2)];
    grad_h_x = [1 0 0 0;
                0 1 0 0];
    grad_h_w = [1 0;
                0 1];
    S = grad_h_w*R*grad_h_w' + grad_h_x*P*grad_h_x';
    W = P*grad_h_x'*inv(S);
    x_hat = x+(W*(z-h)')';
    P_hat = P-W*S*W';
end

