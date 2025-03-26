clear; clc; close all;
%%  特徵匹配
 % 豐原地圖初始化
 TL_coor = [120.7158570, 24.2630837]; % 地圖四個角經緯度，經度先
 TR_coor = [120.7223298, 24.2630837];
 BL_coor = [120.7158570, 24.2573723];
 BR_coor = [120.7223298, 24.2573723];
 folderPath_map = ('map/8KUHD_FongYuan2.png'); % 豐原
 map = imread(folderPath_map);
 resizeFactor = 0.3;

 figure(1)
 title('Real-Time Drone Trajectory');
 imshow(map)
 hold on;
 
 mapWidth = 4103; 
 mapHeight = 3894;
 % 計算一個像素是多少經緯度
 Lon_per_pix = (round(TR_coor(1),8) - round(TL_coor(1),8)) / mapWidth;
 Lat_per_pix = (round(TL_coor(2),8) - round(BL_coor(2),8)) / mapHeight;
 % 地圖水平/垂直距離算
 R = 6378137; % 地球半徑（米）
 dLat = deg2rad(TL_coor(2) - BL_coor(2));
 dLon = deg2rad(TR_coor(1) - TL_coor(1));
 % 平均緯度 (度)，用於水平距離換算
 latMean = (TL_coor(2) + BL_coor(2)) / 2;
 latMean_rad = deg2rad(latMean);
 % 垂直距離（米）
 d_vertical = R * dLat;
 % 水平距離（米）
 d_horizontal = R * cos(latMean_rad) * dLon;
 % 像素與距離的關係(px/m)
 dpx_x = mapWidth / d_horizontal;
 dpx_y = mapHeight / d_vertical;
 
 %% 光流
% 初始參數設置
% DJI Mini2 SE 使用 1/2.3 吋感光元件(6.17mm x 4.55 mm) 4:3
sizeCMOS = [6.17, 4.55]; % 感光元件物理尺寸(長 x 寬 mm)
videoResolution = [2720, 1530]; % 解析度
f = 4.5; % 相機焦距(mm)
H_uav = 120; % 無人機高度(m)
diff_px = sizeCMOS(1) / (videoResolution(1) * resizeFactor); % 實際在感光元件上水平方向像素尺寸(mm/px)，計算光流時影像壓縮成0.3倍，需將resizeFactor考慮進去
diff_py = sizeCMOS(1) * (9/16) / (videoResolution(2) * resizeFactor); % 實際在感光元件上垂直方向像素尺寸(mm/px) 以6.17mm為基準裁成16:9

% 載入影像文件
videoFile = 'video/DJI_0639.MP4'; % 16:9
videoReader = VideoReader(videoFile);

% 讀取影像飛行日誌(10hz)
flightRecord = readtable('log/flight_data.xlsx', 'Sheet', 'video639');
recordPitch =  flightRecord(:,8); % deg
recordRoll =  flightRecord(:,9); % deg
recordYaw =  flightRecord(:,11); % deg[360]
recordVx = flightRecord(:,14); % m/s
recordVy = flightRecord(:,15);
recordPosition = [flightRecord(:,4), flightRecord(:,5)];
recordTime =  flightRecord{:,3} - flightRecord{1,3}; % s 換成從0秒開始

% 將位置換成圖上的像素位置(影片每幀在地圖上的位置)
recordPosition = [ ((flightRecord{:,5} - round(TL_coor(1),8)) / Lon_per_pix), ...
                   ((round(TL_coor(2),8) - flightRecord{:,4}) / Lat_per_pix)];

% 影像播放器
videoPlayer = vision.VideoPlayer;

% 取得第一幀
frame1 = readFrame(videoReader);
resizeFrame1 = imresize(frame1, resizeFactor);
grayFrame1 = rgb2gray(resizeFrame1);

% 在第一幀偵測特徵點(用在接下來追蹤光流)
pointsA = detectKAZEFeatures(grayFrame1);
pointsA = pointsA.selectStrongest(300);

% Harris 角點（避免點集中在邊緣）
harrisPoints = detectHarrisFeatures(grayFrame1);
harrisPoints = harrisPoints.selectStrongest(100);

pointsA_loc = pointsA.Location;
harrisPoints_loc = harrisPoints.Location;

% 合併兩種特徵點
pointsA_loc = [pointsA_loc; harrisPoints_loc];

% 建立point tracker(其內容已含 Lucas-Kanade 方法 - 稀疏光流)
tracker = vision.PointTracker('MaxBidirectionalError', 2);
initialize(tracker, pointsA_loc, grayFrame1);

% 初始化位置(需改成失去GPS第一點的座標)
init_pos = [flightRecord{1,4}, flightRecord{1,5}];

% 無人機初始位置在地圖上的像素位置
init_pos_px = [ (init_pos(2) - round(TL_coor(1),8)) / Lon_per_pix % 初始位置，保持值不變
                (round(TL_coor(2),8) - init_pos(1)) / Lat_per_pix ];

% 取得飛行數據(拍攝影片當下)
yaw = deg2rad(recordYaw{1,1});
yaw_prev = yaw;
roll = deg2rad(recordRoll{1,1});
pitch = deg2rad(recordPitch{1,1});
roll_prev = roll;
pitch_prev = pitch;

% 光流參數設定
processed_fram_rate = 5; % 幾幀處理一次
deltaposX = 0; deltaposY = 0;
uav_p_mapx = init_pos_px(1); uav_p_mapy = init_pos_px(2); % 拿來更新的
X_imu = init_pos_px(1); Y_imu = init_pos_px(2);
uav_true_x = recordPosition(1,1); uav_true_y = recordPosition(1,2);
vX = 0; vY = 0;
tableIdx = 0;
trajectory = [uav_p_mapx, uav_p_mapy];  % 用來儲存每一幀的位置(座標系為何?)
uav_trajectory = [uav_true_x, uav_true_y]; % 真實飛行資料
delta_result = [sqrt( (uav_p_mapx-uav_true_x)^2 + (uav_p_mapy-uav_true_y)^2) ];
dt = 0; 
t_prev = 0;

% 設定畫框參數
hPlot = plot(trajectory(:,1), trajectory(:,2), 'o', 'MarkerSize', 5, 'MarkerFaceColor','b');
hold on
sPlot = plot(uav_trajectory(:,1), uav_trajectory(:,2), '--o', 'MarkerSize', 2, 'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');

frameIdx = 1; % 處理跳幀讀取

%% EKF
% EKF 初始化
% 初始狀態
X = [init_pos_px(1); init_pos_px(2); yaw; 0; 0]; % [X, Y, yaw, vX, vY]

% 狀態協方差矩陣(初始不確定性)
P = [1  0  0  0  0;
     0  1  0  0  0;
     0  0  1  0  0;
     0  0  0  1  0;
     0  0  0  0  1];
% IMU noise
% Q = diag([0.01, 0.01, 0.1, 0.1, 0.1]);
Q = diag([0.002, 0.002, 0.1, 0.01, 0.01]);
% 測量矩陣(光流x y變化量)
H = [1 0 0 0 0;
     0 1 0 0 0];
% 測量噪聲協方差(光流誤差/不確定性)
R_flow = diag([50, 50]);


%% 逐幀讀取
while hasFrame(videoReader)
    frame2 = readFrame(videoReader);
    t = videoReader.CurrentTime;
    % 找出 flightTime 中與 currentTime 差異最小的索引
    [~, tableIdx] = min(abs(recordTime(:) - t));
    
    % 定義幾幀跑一次
    if mod(frameIdx, processed_fram_rate) == 1 && frameIdx > processed_fram_rate
        
        dt = t - t_prev;
        resizeFrame2 = imresize(frame2, resizeFactor);
        grayFrame2 = rgb2gray(resizeFrame2);

        % Point tracker
        [pointsB_loc, isFound] = step(tracker, grayFrame2);
        % isFound是確認各個特徵點是否有成功追蹤到的flag

        % 篩選有效點
        validOldPoints = pointsA_loc(isFound, :);
        validNewPoints = pointsB_loc(isFound, :);

        % RANSAC 仿射變換(去除錯誤點)
        [tform, inlierIdx] = estimateGeometricTransform2D(...
                              validOldPoints, validNewPoints, ...
                              'affine', 'MaxDistance', 2);
                          
        
                          
        % 仿射變換矩陣
        A = tform.T;            
        
        % 當前幀的IMU資訊
        yaw = deg2rad(recordYaw{tableIdx,1}); 
        pitch = deg2rad(recordPitch{tableIdx,1});
        roll = deg2rad(recordRoll{tableIdx,1});
        vX = recordVy{tableIdx,1};
        vY = recordVx{tableIdx,1};
        
        
        % 無人機的yaw，從正北順時針轉yaw度
        R_yaw = [ cos(yaw), sin(yaw);
                 -sin(yaw), cos(yaw)];
             
        % 更新追蹤器:只保留 inlier 的新舊點
        inlierOld = validOldPoints(inlierIdx, :);
        inlierNew = validNewPoints(inlierIdx, :);
        setPoints(tracker, inlierNew);
        
        % 計算位移(經過yaw補償)
        cx = size(grayFrame2,2) / 2;
        cy = size(grayFrame2,1) / 2;

        delta_yaw = yaw - yaw_prev;
        delta_roll = roll - roll_prev;
        delta_pitch = pitch - pitch_prev;
        rel_old = inlierOld - [cx, cy]; % inlier 每一點相對於中心的向量
        
        rot_disp_yaw = zeros(size(rel_old));
        
        % 姿態補償
%         fx = (f/sizeCMOS(1))*videoResolution(1)*resizeFactor;
%         fy = (f/sizeCMOS(2))*videoResolution(2)*resizeFactor;
%         
%         K = [fx 0 cx;
%              0 fy cy;
%              0  0  1]; % 相機內部參數矩陣
%          
%         Rx = [1 0 0;
%               0 cos(delta_roll) -sin(delta_roll);
%               0 sin(delta_roll) cos(delta_roll)];
%           
%         Ry = [cos(delta_pitch) 0 -sin(delta_pitch);
%               0 1 0;
%               sin(delta_pitch) 0 cos(delta_pitch)];
%            
%         Rz = [cos(delta_yaw) sin(delta_yaw) 0;
%               -sin(delta_yaw)  cos(delta_yaw) 0; % delta_yaw為順時針算
%               0 0 1];
%            
%         R = Rz*Ry*Rx;
%         
%         H_comp = K*R*inv(K);
%         
%         N = size(inlierOld, 1);
%         
%         p_rot = zeros(N, 2);
%         
%         for i = 1:N
%             pt = [inlierOld(i,1); inlierOld(i,2); 1];   % 擴充成齊次座標
%             pt_trans = H_comp * pt;           % 透過 Homography 變換
%             pt_trans = pt_trans / pt_trans(3); % 齊次化
%             p_rot(i,:) = pt_trans(1:2)';
%         end
%         
%         d_rot = p_rot - inlierOld;
        
        % yaw 補償
        yaw_rot = [cos(delta_yaw), sin(delta_yaw);  % yaw定義為CW+，與旋轉矩陣定義不同
                   -sin(delta_yaw),  cos(delta_yaw)];
        rot_disp_yaw = (yaw_rot * rel_old')' - rel_old; % 向量相減
        
        % 考慮鏡頭偏移補償
        cam_offset = 0.1; % m
        cam_disp = cam_offset * [sin(yaw) - sin(yaw_prev), cos(yaw) - cos(yaw_prev)]; % m; 向量相減
        cam_disp_px = [cam_disp(1) * dpx_x, cam_disp(2) * dpx_y];
        
        % Pitch & Roll 補償 (改進 Homography 方法)
        H_pitch = [1, 0, 0;
                   0, 1, tan(delta_pitch);
                   0, 0, 1];

        H_roll = [1, 0, tan(delta_roll);
                  0, 1, 0;
                  0, 0, 1];

        H_total = H_pitch * H_roll;
        rel_old_homog = [rel_old, ones(size(rel_old,1), 1)]'; % 擴展成齊次座標
        corrected_points = (H_total * rel_old_homog)'; % 進行投影變換
        corrected_points = corrected_points(:,1:2) ./ corrected_points(:,3); % 轉回 2D 坐標
       
        displacement = (inlierNew - inlierOld);

        
%         displacement = displacement + rot_disp_yaw + cam_disp_px + (corrected_points - rel_old);
        displacement(:,1) = displacement(:,1) - rot_disp_yaw(:,1) - cam_disp_px(:,1) - (corrected_points(:,1) - rel_old(:,1));
        displacement(:,2) = displacement(:,2) - rot_disp_yaw(:,2) - cam_disp_px(:,2) - (corrected_points(:,2) - rel_old(:,2));
%         displacement = displacement + rot_disp_yaw;

        
        % 計算光流平均位移
        meanDisplacement = mean(displacement, 1); % 平均值
%         meanDisplacement = median(displacement, 1);  % 中間值
        diff_p_body = [meanDisplacement(1), meanDisplacement(2)]; % 體座標下光流估算的位移(px)
        diff_p_global = R_yaw * diff_p_body';  % 光流慣性座標下位移(px)
        % 計算軌跡
        deltaposX = diff_p_global(1); % pixel
        deltaposY = diff_p_global(2); % pixel
        % 轉成實際尺寸
        dx_mm = deltaposX * diff_px; % 將像素位移轉換為感光元件上的位移 (mm)
        dy_mm = deltaposY * diff_py;
        ground_x = H_uav * (dx_mm / f); % 轉換為地面上實際位移(m)
        ground_y = H_uav * (dy_mm / f);
        
        % 轉換到地圖座標(m -> px) (慣性座標)
%         uav_p_mapx = uav_p_mapx + ground_x * dpx_x;
%         uav_p_mapy = uav_p_mapy - ground_y * dpx_y; % imshow往下為正
        
        % 計算內點比率
        inlierRatio = size(inlierIdx, 1) / size(validOldPoints, 1);
        
        % 計算角速率
        yaw_rate = abs(rad2deg((yaw - yaw_prev) / dt));
        pitch_rate = abs(rad2deg(pitch - pitch_prev) / dt);
        roll_rate = abs(rad2deg(roll-roll_prev) / dt);
        
        
        % EKF
        delta_flow = [deltaposX; deltaposY]; % 慣性座標(單位為px)
        imu_State = [yaw; pitch; roll; vX; vY];
        % 這裡可設條件動態調整Q跟R_flow
        % 依據yaw角速率動態調整 Q
%         Q = diag( [min(0.001, 1/(yaw_rate*100)), ...  % X 位置noise
%                    min(0.001, 1/(yaw_rate*100)), ...  % Y 位置noise
%                    0.1, ... % Yaw
%                    0.001, ... % Vx
%                    0.001]  );  % Vy
%         % 依據yaw角速率與inlier數量動態調整 R_flow
%         R_flow = diag( [(max(50, yaw_rate * 15 + 50 / size(inlierNew,1))) / inlierRatio, ...   % X 方向光流noise
%                         (max(50, yaw_rate * 15 + 50 / size(inlierNew,1))) / inlierRatio] );    % Y 方向光流noise
     
        [X_pre, X, P] = EKF_Predict(X, P, Q, dt, imu_State);
        [X, P] = EKF_Update(X, X_pre, P, H, delta_flow, R_flow);
        
        % 理論上這邊計算出來的X(1), X(2)就會是慣性座標上的點
        uav_p_mapx = X(1);
        uav_p_mapy = X(2);
   
        
        % 記錄軌跡
        trajectory(end+1,:) = [uav_p_mapx, uav_p_mapy];
        uav_trajectory(end+1,:) = [recordPosition(tableIdx,1), recordPosition(tableIdx,2)];
        dx_m  = (uav_p_mapx - recordPosition(tableIdx,1)) / dpx_x;
        dy_m  = (uav_p_mapy - recordPosition(tableIdx,2)) / dpx_y;
        delta_result(end+1,1) = [ sqrt(dx_m^2 + dy_m^2) ];
        
        % 更新迴圈參數
        pointsA_loc = inlierNew;
        yaw_prev = yaw;
        pitch_prev = pitch;
        roll_prev = roll;
        t_prev = t;
        
        % 軌跡畫在地圖上
        set(hPlot, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
        set(sPlot, 'XData', uav_trajectory(:,1), 'YData', uav_trajectory(:,2));
        drawnow;
        
        % 如果有效點太少，重新偵測當前幀特徵，同時做KAZE特徵匹配
        if size(validNewPoints,1) < 50
            
            % 全局特徵匹配校正光流累積誤差
            [uav_p_map, inlierRatio, meanError, confidence, rotation] = kazeFeatureMatching_fcn([uav_p_mapx,uav_p_mapy], grayFrame2, yaw, pitch, roll, flightRecord{tableIdx,4});
            % 若特徵匹配計算出來的距離與光流估算的位置差太多則不接受
            if sqrt( (X(1)-uav_p_map(1))^2 + (X(2)-uav_p_map(2))^2 ) < 100 || abs(rotation) > 45
                % 動態融合
                epsilon = 1e-3;
                conf_feat = inlierRatio / (meanError/100 + epsilon);
                conf_ekf = 1; 
                w_feat = conf_feat*3 / (conf_feat + conf_ekf)
                w_ekf = 1 - w_feat
                X(1) = w_feat * uav_p_map(1) + w_ekf * X(1);
                X(2) = w_feat * uav_p_map(2) + w_ekf * X(2);
                disp('進行特徵匹配與EKF動態融合')
            else
                disp('特徵匹配與光流相差太多')
            end
            
            
            
            % 重新偵測+初始化
            release(tracker);
%             pointsA = detectMinEigenFeatures(grayFrame2, 'MinQuality', 0.005);
%             pointsA = detectSURFFeatures(grayFrame1, 'MetricThreshold', 300);
            pointsA = detectKAZEFeatures(grayFrame2);
            pointsA = pointsA.selectStrongest(300);
            
            % Harris 角點（避免點集中在邊緣）
            harrisPoints = detectHarrisFeatures(grayFrame1);
            harrisPoints = harrisPoints.selectStrongest(100);
            
            pointsA_loc = pointsA.Location;
            harrisPoints_loc = harrisPoints.Location;

            % 合併兩種特徵點
            pointsA_loc = [pointsA_loc; harrisPoints_loc];
    
            pointsA_loc = pointsA.Location;
            initialize(tracker, pointsA_loc, grayFrame2);
            disp('重新偵測特徵點');
        end

        % 在影像上可視化光流向量 -----------------
        outFrame = resizeFrame2; % 複製當前影像以疊加視覺化
        
        for i = 1:10:size(validNewPoints,1)
            
            ptOld = validOldPoints(i,:);
            ptNew = validNewPoints(i,:);
            % 1) 畫舊點: 紅色小圓
            outFrame = insertShape(outFrame, 'FilledCircle', [ptOld, 3], ...
                'Color', 'red', 'Opacity', 0.7);
            % 2) 畫線: 從舊點→新點
            outFrame = insertShape(outFrame, 'Line', [ptOld ptNew], ...
                'Color', 'yellow', 'Opacity', 0.9);
            % 3) 畫新點: 綠色小圓
            outFrame = insertShape(outFrame, 'FilledCircle', [ptNew, 3], ...
                'Color', 'green', 'Opacity', 0.7);
            
        end

        % 顯示處理後影像
        step(videoPlayer, outFrame); 
    end
    
    frameIdx = frameIdx + 1;
    dync_fusion_flag = 0;
end

% 釋放videoPlayer
release(videoPlayer);





