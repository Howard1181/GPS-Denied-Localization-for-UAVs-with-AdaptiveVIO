%% 狀態更新
function [X_upd, P_upd] = EKF_Update(X, X_pre, P_pre, H, delta_flow, R_flow)

    % 量測模型 H
    % R_flow 是光流測量協方差

    % 測量向量(光流在慣性座標的位移)
    flow_noise = 0.05 * randn(2,1); % 光流測量誤差
    delta_flow = delta_flow + flow_noise;
    % z = H * X_pre + delta_flow;
    z = [X(1) + delta_flow(1);
         X(2) - delta_flow(2)];

    % 計算殘差
    y = z - H * X_pre;
    residual_norm = norm(y);
    threshold = 2; 

    % 計算卡爾曼增益
    K = P_pre * H' * inv(H * P_pre * H' + R_flow);

    if residual_norm > threshold
        X_upd = X_pre; % 若residual_norm大於閥值則不接受光流修正
        P_upd = P_pre;
         disp(['殘差過大，不信任光流量測']);
    else
        % 更新狀態估計
        X_upd = X_pre + K * y;
        % 更新誤差協方差
        P_upd = (eye(5) - K * H) * P_pre;
    end
    
end