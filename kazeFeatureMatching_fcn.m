%% KAZE特徵匹配
function [out, inlierRatio, meanError, confidence, rotation]  = kazeFeatureMatching_fcn(uav_pred_pos, img, flight_yaw, flight_pitch, flight_roll, lat_now)
%%
    % 豐原
    Map_Width = 4103; 
	Map_Height = 3894;
    TL_coor = [120.7158570, 24.2630837]; 
    TR_coor = [120.7223298, 24.2630837];
    BL_coor = [120.7158570, 24.2573723];
    BR_coor = [120.7223298, 24.2573723];
    CropSize = 300;
    numrow = ceil(Map_Width / CropSize) * CropSize; % 將numrow往上取到CropSize的倍數
    numcol = ceil(Map_Height / CropSize) * CropSize;
    Lon_per_pix = (round(TR_coor(1),8) - round(TL_coor(1),8)) / Map_Width;  % 這些應要都是已知值
    Lat_per_pix = (round(TL_coor(2),8) - round(BL_coor(2),8)) / Map_Height;
    
    % 計算拼接圖片中心
    block_X = ceil(abs(uav_pred_pos(1)) / CropSize);
    block_Y = ceil(abs(uav_pred_pos(2)) / CropSize);
    picture_num = (numcol/CropSize)*(block_X-1) + block_Y; % 座標點會落在picture_num這張小圖片上，圖片編號由上而下、由左至右編號

    % 從資料夾選取拼接圖片
    CropImg_folderPath = 'Cropped_Map_FU'; % 豐原
    CropimgName = sprintf('%d.png', picture_num);
    CropimgPath = fullfile(CropImg_folderPath, CropimgName);
    if exist(CropimgPath, 'file')
        selectedImg = imread(CropimgPath);
    else
        disp('指定的圖片檔案不存在');
    end
    
    % 拼接小圖週遭地圖
    xStepNum = floor((numrow-CropSize)/CropSize+1); % 朝負無窮方向取整，寬度方向block移動的次數
    yStepNum = floor((numcol-CropSize)/CropSize+1); % 朝負無窮方向取整，長度方向block移動的次數
    xyStepNum = xStepNum*yStepNum;
    
    % 拼 3*3
    % 將小圖片及周遭的8張一併讀取出來，考量可能會抓到邊界或是計算到的圖片編號不在範圍內的情況，若發生這種現象則用零取代
    if (picture_num-yStepNum-1)<1 || (picture_num-yStepNum-1)>xyStepNum
        MapNum1 = zeros([CropSize,CropSize,3]);
    else
        MapNum1 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num-yStepNum-1),'.png']));
    end

    if (picture_num-yStepNum)<1 || (picture_num-yStepNum)>xyStepNum
        MapNum2 =zeros([CropSize,CropSize,3]);
    else
        MapNum2 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num-yStepNum),'.png']));
    end

    if (picture_num-yStepNum+1)<1 || (picture_num-yStepNum+1)>xyStepNum
        MapNum3 =zeros([CropSize,CropSize,3]);
    else
        MapNum3 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num-yStepNum+1),'.png']));
    end

    if (picture_num-1)<1 || (picture_num-1)>xyStepNum
        MapNum4 =zeros([CropSize,CropSize,3]);
    else
        MapNum4 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num-1),'.png']));
    end

    if (picture_num)<1 || (picture_num)>xyStepNum
        MapNum5 =zeros([CropSize,CropSize,3]);
    else
        MapNum5 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num),'.png']));
    end

    if (picture_num+1)<1 || (picture_num+1)>xyStepNum
        MapNum6 =zeros([CropSize,CropSize,3]);
    else
        MapNum6 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num+1),'.png']));
    end

    if (picture_num+yStepNum-1)<1 || (picture_num+yStepNum-1)>xyStepNum
        MapNum7 =zeros([CropSize,CropSize,3]);
    else    
        MapNum7 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num+yStepNum-1),'.png']));
    end

    if (picture_num+yStepNum)<1 || (picture_num+yStepNum)>xyStepNum
        MapNum8 =zeros([CropSize,CropSize,3]);
    else    
        MapNum8 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num+yStepNum),'.png']));
    end

    if (picture_num+yStepNum+1)<1 || (picture_num+yStepNum+1)>xyStepNum
        MapNum9 =zeros([CropSize,CropSize,3]);
    else
        MapNum9 = im2double(imread(['Cropped_Map_FU\',num2str(picture_num+yStepNum+1),'.png']));
    end
    
    % 避免上下邊界的誤判
    for  count = 1:1:xStepNum % 第一排和最後一排分別有11個數字要判斷  
        % 若 picture_num 的值位於第一排
        if picture_num == (count-1)*yStepNum+1
            % 就把 MapNum1、MapNum4 和 MapNum7 用零矩陣替換掉
            MapNum1 = zeros([CropSize,CropSize,3]);  
            MapNum4 = zeros([CropSize,CropSize,3]);   
            MapNum7 = zeros([CropSize,CropSize,3]);
        end
        % 若 picture_num 的值位於最末排
        if picture_num == (count)*yStepNum
            % 就把 MapNum3、MapNum6 和 MapNum9 用零矩陣替換掉
            MapNum3 = zeros([CropSize,CropSize,3]);  
            MapNum6 = zeros([CropSize,CropSize,3]);   
            MapNum9 = zeros([CropSize,CropSize,3]);
        end
    end

    % 再把九張圖片照順序拼回去
    Puzzle_c = [MapNum1 MapNum4 MapNum7; MapNum2 MapNum5 MapNum8; MapNum3 MapNum6 MapNum9];
    
    % 照片姿態補償
    
    
    % 畫圖
    % 把拼好的圖片顯示出來
    figure(5);
    subplot(2,1,1);
    imshow(double(Puzzle_c));
    title('拼接完後的圖片[3*3]')
    subplot(2,1,2);
    imshow(img);
    title('無人機空拍圖')
    
    % 圖片前處理
%     sfactor2 = 0.3; %豐原 
%     img = imresize(img, sfactor2);
    img = imrotate(img, -rad2deg(flight_yaw)); % 順時針轉
    
%     R_yaw = [cos(flight_yaw) sin(flight_yaw)  0; 
%              -sin(flight_yaw) cos() 0;
%               0 0 1];
%           
%     R_pitch = [cos(flight_pitch) 0 sin(flight_pitch);
%                0 1 0;
%                -sin(flight_pitch) 0 cos(flight_pitch)];
%            
%     R_roll = [1 0 0;
%               0 cos(flight_roll) -sin(flight_roll)
%               0 sin(flight_roll)  cos(flight_roll)];   
%           
%     R = R_yaw * R_pitch * R_roll;
%     H = K * R * inv(K);
%     tform = projective2d(H');
%     img = imwarp(img, tform, 'OutputView', imref2d(size(img)));
    
    % 圖片轉灰階
    Gray_img1 = rgb2gray(Puzzle_c);
    Gray_img2 = img;
    
    % 判斷是否需調整亮度/對比
%     Gray_img2_db = double(Gray_img2) / 255;  % 將uint8圖片轉換成雙精度
%     mean1 = mean(Gray_img1(:));
%     mean2 = mean(Gray_img2_db(:));
%     std1 = std(Gray_img1(:));
%     std2 = std(Gray_img2_db(:));
%     disp(['Image 1 亮度: ', num2str(mean1), ', 對比度: ', num2str(std1)]);
%     disp(['Image 2 亮度: ', num2str(mean2), ', 對比度: ', num2str(std2)]);
% 
%     if (mean1 > 0.2 && abs(mean1-mean2) > 0.2 && abs(mean1-mean2) < 0.3) || (mean1 > 0.2 && abs(std1-std2) > 0.15 && abs(std1-std2) < 0.25)
%         Gray_img2 = imhistmatch(Gray_img2, Gray_img1); % 光度均一化
%         Gray_img2_db = double(Gray_img2) / 255;
%         mean1 = mean(Gray_img1(:));
%         mean2 = mean(Gray_img2_db(:));
%         std1 = std(Gray_img1(:));
%         std2 = std(Gray_img2_db(:));
%         disp('亮度/對比差異大，執行光度均一');
%         disp(['Image 1 均一化亮度: ', num2str(mean1), ', 均一化對比度: ', num2str(std1)]);
%         disp(['Image 2 均一化亮度: ', num2str(mean2), ', 均一化對比度: ', num2str(std2)]);
%     end
    
    % 特徵檢測
    points_img1 = detectKAZEFeatures(Gray_img1);
    points_img2 = detectKAZEFeatures(Gray_img2);
    points_img1 = points_img1.selectStrongest(15000);
    points_img2 = points_img2.selectStrongest(15000);
    
    % 計算描述符
    [f1, vpts1] = extractFeatures(Gray_img1, points_img1);
    [f2, vpts2] = extractFeatures(Gray_img2, points_img2);
    
    % 進行匹配
    indexPairs = matchFeatures(f1, f2, 'MatchThreshold', 20, 'MaxRatio', 0.7) ; 
    
    matched_pts1 = vpts1(indexPairs(:, 1));
    matched_pts2 = vpts2(indexPairs(:, 2));
    
    % RANSAC
    [tform, inlierimg2Points, inlierimg1Points] = estimateGeometricTransform(matched_pts2, matched_pts1, 'similarity', 'MaxNumTrials', 5000, 'MaxDistance', 10);
    
    % 信心程度參數
    numMatches = size(indexPairs, 1);
    numInliers = size(inlierimg1Points, 1);
    inlierRatio = numInliers / numMatches
    errors = zeros(numInliers,1);
    for i = 1:numInliers
        projectedPoint = transformPointsForward(tform, inlierimg1Points.Location(i,:));
        errors(i) = norm(projectedPoint - inlierimg2Points.Location(i,:));
    end
    meanError = mean(errors)
    T = tform.T;
    scale = mean([norm(T(1:2,1)), norm(T(1:2,2))]);
    rotation = rad2deg(atan2(T(2,1), T(1,1)));  % radians
    dispersion = mean(std(inlierimg2Points.Location)); %判斷點是否過於集中
    confidence = (inlierRatio * 5) * (1 / (1 + meanError)) * (1 / (1 + abs(scale - 1))) * (1 / (1 + 100/dispersion));
    confidence = confidence / (confidence + 1)
    
    figure(6)
    showMatchedFeatures(Gray_img1,Gray_img2,inlierimg1Points,inlierimg2Points,'montage');
    legend('matched points 1','matched points 2', 'FontSize', 18);
    title("Feature Matching Results after Applying RANSAC", 'FontSize', 36)
     
    % 疊圖
    Rfixed = imref2d(size(Puzzle_c));
    [registered2, Rregistered] = imwarp(img, tform);
     boxPolygon = [1, 1; ... % 左上
       size(img, 2), 1; ... % 右上
       size(img, 2), size(img, 1); ... % 右下
       1, size(img, 1); ... % 左下
       1, 1]; % 重複左下，才可得到一個閉區間的多邊形
    % 將多邊形變換到目標圖片上，變換的結果表示了物體的位置
    newBoxPolygon = transformPointsForward(tform, boxPolygon);
    figure(7)
    imshowpair(Puzzle_c,Rfixed,registered2,Rregistered,'blend');
    hold on
    Xmdn = (newBoxPolygon(1, 1)+newBoxPolygon(2, 1)+newBoxPolygon(3, 1)+newBoxPolygon(4, 1))/4;
    Ymdn = (newBoxPolygon(1, 2)+newBoxPolygon(2, 2)+newBoxPolygon(3, 2)+newBoxPolygon(4, 2))/4;
    line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');
    hold on;
    InterestPoint1 = scatter(Xmdn, Ymdn, '*b');
    
    % 計算匹配結果照片左上角經緯度
    CropMap_TL_Pix = [CropSize*(block_X-2), CropSize*(block_Y-2)];
    
    % 計算特徵匹配照片的經緯度
    Estimated_Lat = TL_coor(2) - (CropMap_TL_Pix(2) + Ymdn ) * Lat_per_pix;  % -100 -150作弊用不該存在
    Estimated_Lon = TL_coor(1) + (CropMap_TL_Pix(1) + Xmdn ) * Lon_per_pix; 

    total_pitch = flight_pitch; % 豐原用 '+' 逢甲改 '-' 比較準，pitch定義不同
    delta_d = 120 * tan(total_pitch);
    delta_x = delta_d * sin(flight_yaw); % m
    delta_y = delta_d * cos(flight_yaw); % m
    delta_y2lat = 0.00898 * delta_y/1000; % 1000公尺影響 0.00898緯度
    delta_x2lon = (1000/(111320*cosd(lat_now))) * delta_x/1000; % 1000公尺影響 (1000/(111320*cosd(當地緯度))) 的經度，葫蘆墩公園緯度為24.26
%     delta_x2lon = (1000/(111320*cosd(24.17))) * delta_x/1000; % 1000公尺影響 (1000/(111320*cosd(當地緯度))) 的經度，逢甲緯度
%     Estimated_Pos = [Estimated_Lat - delta_y2lat, Estimated_Lon - delta_x2lon];
    Estimated_Pos = [Estimated_Lat Estimated_Lon];
    
    Estimated_pixX = (round(Estimated_Pos(2),8) - round(BL_coor(1),8)) / Lon_per_pix;
    Estimated_pixY = Map_Height - (round(Estimated_Pos(1),8) - round(BL_coor(2),8)) / Lat_per_pix;
    
    out = [Estimated_pixX, Estimated_pixY];
    
    close (5)
    close (6)
    close (7)
end