%% 狀態預測
function [X_pre, X, P_pre] = EKF_Predict(X, P, Q, dt, imu_state)
    % 狀態轉移矩陣
    F = [1 0 0 dt 0 ;
         0 1 0 0  dt;
         0 0 1 0  0 ;
         0 0 0 1  0 ;
         0 0 0 0  1];
    % 更新狀態
    X_pre(1) = X(1) + imu_state(4) * dt * 6.245849865773375; % 要記得換算成像素
    X_pre(2) = X(2) - imu_state(5) * dt * 6.124662458174774;
    X_pre(3) = imu_state(1);
    X_pre(4) = imu_state(4);
    X_pre(5) = imu_state(5);
    
    % 狀態預測
    X_pre = X_pre(:); % 轉為列向量

    % 預測誤差更新
    P_pre = F * P * F' + Q;
end