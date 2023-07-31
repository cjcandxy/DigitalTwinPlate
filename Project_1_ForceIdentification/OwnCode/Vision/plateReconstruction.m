%% 加载初始化数据
clear variables
clc

% 导入先验信息
% romMatrix    - 导入降阶模型矩阵
    % 内部含有transMatrix，定义了局部到全局信息
    % 降阶结果中应包含视觉坐标系到全局坐标系信息
% visionInit   - 定义视觉初始化矩阵
    % stereoMatrix - 双目相机标定矩阵
    % boxInit        - 定义初始目标检测框（拓展）位置
    % coordinateInit - 定义初始坐标位置
    % matchInit      - 定义初始化检测信息

% load romMatrix.mat
load visionInit.mat

stereoMatrix = visionInit.stereoMatrix;
boxInit = visionInit.boxInit;
coordinateInit = visionInit.coordinateInit;
matchInit = visionInit.matchInit;

fname = 'Job-3-1kg-size10.inp';
[Nodes, Elements] = Readmesh(fname);

%% 初始化标志点跟踪及重建系统

% 初始化检测系统对象
area_min = 300; area_max = 1000;
detectorObjects = setupDetectorObjects(area_min,area_max);


%% 初始化跟踪模型

numFrames = 80;

tracker_1 = trackerGNN(MaxNumSensors=1,MaxNumTracks=20);
tracker_1.FilterInitializationFcn = @initcvekf;
% tracker.FilterInitializationFcn = @initFastCAEKF;
tracker_1.ConfirmationThreshold = [2 2]; % Quick to confirm
tracker_1.DeletionThreshold = [40 40];   % Slow to delete
% tracker.AssignmentClustering = 'on';
% tracker.MaxNumTracksPerCluster = 6;
tracker_1.AssignmentThreshold = [100 Inf];

tracker_2 = trackerGNN(MaxNumSensors=1,MaxNumTracks=20);
tracker_2.FilterInitializationFcn = @initcvekf;
% tracker.FilterInitializationFcn = @initFastCAEKF;
tracker_2.ConfirmationThreshold = [2 2]; % Quick to confirm
tracker_2.DeletionThreshold = [40 40];   % Slow to delete
% tracker.AssignmentClustering = 'on';
% tracker.MaxNumTracksPerCluster = 6;
tracker_2.AssignmentThreshold = [100 Inf];

frameCount = 0;
% vidReader.CurrentTime = 0; % Reset the video reader
vidPlayer = vision.DeployableVideoPlayer("Size","Full-screen");
 
bboxes_1 = cell(1,numFrames);
centroids_1 = cell(1,numFrames);
detectionHistory_1 = cell(1,numFrames);

bboxes_2 = cell(1,numFrames);
centroids_2 = cell(1,numFrames);
detectionHistory_2 = cell(1,numFrames);

trackers_1 = cell(1,numFrames);
trackers_2 = cell(1,numFrames);
%%
v1 = videoinput("gentl", 1, "Mono8");
v2 = videoinput("gentl", 2, "Mono8");
v1.FramesPerTrigger = 100;
v2.FramesPerTrigger = 100; 

coordinate_int = load('coorInit.mat').coordinate;

xyzInit = coordinate_int;

% h = scatter3(xyzInit(:,1), xyzInit(:,2), xyzInit(:,3), 'filled', 'MarkerFaceColor', 'b');

% U_int = zeros(1080,1);
% h = patch('Vertices',Nodes,'Faces',Elements,'FaceVertexCData',U_int,'FaceColor','interp','EdgeAlpha',0);
% view(3);
% 
% clim([0,20]);
% t1=clim;
% t1=linspace(t1(1),t1(2),13);
% colorbar('ytick',t1,'Location','westoutside');
% colormap(jet);
% axis equal
% axis off

start(v1);
start(v2);

% %关闭之前存在UDP端口
% fclose(instrfindall);
% % 创建UDP对象，设置IP地址和端口
% udpSender = udp('192.168.43.116', 'RemotePort',8401,'LocalPort',8400);
% % 打开UDP对象
% fopen(udpSender);
bbPredict_1 = boxInit.box_1;
bbPredict_2 = boxInit.box_2;
%% 

for i = 1:numFrames

    while ~(v1.FramesAvailable && v2.FramesAvailable)
        pause(0.001);
    end   
    
    frame_c1 = getsnapshot(v1);  
    frame_c2 = getsnapshot(v2); 

    frameCount = frameCount + 1; % Increment frame count
    
    [centroids_1{frameCount}, bboxes_1{frameCount}] = detectBlobs(detectorObjects, frame_c1, bbPredict_1);
    [centroids_2{frameCount}, bboxes_2{frameCount}] = detectBlobs(detectorObjects, frame_c2, bbPredict_2);
    
    thisFrameCentroids_1 = centroids_1{frameCount};
    thisFrameCentroids_2 = centroids_2{frameCount};

    thisFrameBboxes_1 = bboxes_1{frameCount};
    thisFrameBboxes_2 = bboxes_2{frameCount};
    

    %%%%%%%%%%%%%%%%%%%


    [trackers_1{frameCount},bbPredict_1_track,cen_1] = markPointTracking(frameCount,thisFrameCentroids_1,thisFrameBboxes_1,tracker_1);
    [trackers_2{frameCount},bbPredict_2_track,cen_2] = markPointTracking(frameCount,thisFrameCentroids_2,thisFrameBboxes_2,tracker_2);
    
    
    if frameCount>1
        bbPredict_1 = bbPredict_1_track;
        bbPredict_2 = bbPredict_2_track;
    end
       
    cen_11 = centroids_1{frameCount};
    cen_22 = centroids_2{frameCount};
    
    coordinate = coordCalculate(cen_11,cen_22,stereoMatrix);
    
    displacement = coordinate - coordinate_int;
    
    displacement_sum = sqrt(sum(displacement.^2, 2));
    
%————————————————————
%     % 传输工作区数据
%     data = displacement_sum(1); 
%     fwrite(udpSender, data, 'double'); % 传输双精度浮点数数据
%————————————————————

    displacement_sum_new = zeros(6,1);
    
    displacement_sum_new(1:2,:) = displacement_sum(5:6,:);
    displacement_sum_new(3:4,:) = displacement_sum(3:4,:);
    displacement_sum_new(5:6,:) = displacement_sum(1:2,:);
    
    displacementFullfiled = transMatrix(1:3240,:)*displacement_sum_new;
  
%     set(h,"FaceVertexCData",displacementFullfiled(2:3:end));
%     drawnow limitrate nocallbacks

%     x = coordinate(:,1);
%     y = coordinate(:,2);
%     z = coordinate(:,3);
%     set(h,"XData",x,"YData",y,"ZData",z);   
% 
%     frame = insertObjectAnnotation(frame, "rectangle", boxes, labels, ...
%          TextBoxOpacity = 0.0,LineWidth=5,Color="red",TextColor="red",FontSize = 48);
    
    % frame_c1 = insertShape(frame_c1,"rectangle",thisFrameBboxes_1,"LineWidth",4,"Color","blue");
    frame_c1 = insertShape(frame_c1,"rectangle",bbPredict_1_track,"LineWidth",4,"Color","cyan");
    
    % Display Video
    step(vidPlayer,frame_c1);   

end

stop(v1)
delete(v1);
stop(v2)
delete(v2);
% release(vidPlayer);