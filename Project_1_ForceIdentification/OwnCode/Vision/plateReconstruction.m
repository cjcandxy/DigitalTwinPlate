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

numFrames = 20;

tracker = trackerGNN(MaxNumSensors=1,MaxNumTracks=20);
tracker.FilterInitializationFcn = @initcvekf;
% tracker.FilterInitializationFcn = @initFastCAEKF;
tracker.ConfirmationThreshold = [2 2]; % Quick to confirm
tracker.DeletionThreshold = [40 40];   % Slow to delete
% tracker.AssignmentClustering = 'on';
% tracker.MaxNumTracksPerCluster = 6;
tracker.AssignmentThreshold = [100 Inf];

frameCount = 0;
% vidReader.CurrentTime = 0; % Reset the video reader
vidPlayer = vision.DeployableVideoPlayer("Size","Full-screen");
 
bboxes_1 = cell(1,numFrames);
centroids_1 = cell(1,numFrames);
detectionHistory_1 = cell(1,numFrames);

bboxes_2 = cell(1,numFrames);
centroids_2 = cell(1,numFrames);
detectionHistory_2 = cell(1,numFrames);


%%
v1 = videoinput("gentl", 1, "Mono8");
v2 = videoinput("gentl", 2, "Mono8");
v1.FramesPerTrigger = 30;
v2.FramesPerTrigger = 30; 

coordinate_int = load('coorInit.mat').coordinate;
% h = scatter3(xyzInit(:,1), xyzInit(:,2), xyzInit(:,3), 'filled', 'MarkerFaceColor', 'b');

U_int = zeros(1080,1);
h = patch('Vertices',Nodes,'Faces',Elements,'FaceVertexCData',U_int,'FaceColor','interp','EdgeAlpha',0);
view(3);

clim([0,20]);
t1=clim;
t1=linspace(t1(1),t1(2),13);
colorbar('ytick',t1,'Location','westoutside');
colormap(jet);
axis equal
axis off

start(v1);
start(v2);


for i = 1:numFrames

    while ~(v1.FramesAvailable && v2.FramesAvailable)
        pause(0.001);
    end   
    
    frame_c1 = getsnapshot(v1);  
    frame_c2 = getsnapshot(v2); 

    frameCount = frameCount + 1; % Increment frame count

    [centroids_1{frameCount}, bboxes_1{frameCount}] = detectBlobs(detectorObjects, frame_c1, boxInit.box_1);
    [centroids_2{frameCount}, bboxes_2{frameCount}] = detectBlobs(detectorObjects, frame_c2, boxInit.box_2);
    
    thisFrameCentroids_1 = centroids_1{frameCount};
    thisFrameCentroids_2 = centroids_2{frameCount};

    thisFrameBboxes_1 = bboxes_1{frameCount};
    thisFrameBboxes_2 = bboxes_2{frameCount};
    
    % [bbPredict_1,cen_1] = markPointTracking(frameCount,thisFrameCentroids_1,thisFrameBboxes_1,tracker);

    
    cen_11 = centroids_1{frameCount};
    cen_22 = centroids_2{frameCount};
    
    coordinate = coordCalculate(cen_11,cen_22,stereoMatrix);
    
    displacement = coordinate - coordinate_int;
    
    displacement_sum = sqrt(sum(displacement.^2, 2));
    
    displacement_sum_new = zeros(6,1);

    displacement_sum_new(1:2,:) = displacement_sum(5:6,:);
    displacement_sum_new(3:4,:) = displacement_sum(3:4,:);
    displacement_sum_new(5:6,:) = displacement_sum(1:2,:);
    
    displacementFullfiled = transMatrix(1:3240,:)*displacement_sum_new;
  
    set(h,"FaceVertexCData",displacementFullfiled(2:3:end));
    drawnow limitrate nocallbacks
    % x = coordinate(:,1);
    % y = coordinate(:,2);
    % z = coordinate(:,3);
    % set(h,"XData",x,"YData",y,"ZData",z);   
        

end

stop(v1)
delete(v1);
stop(v2)
delete(v2);

% release(vidPlayer);

%% 
function [centroids, bbox] = detectBlobs(detectorObjects, frame, bbPredict)
% Expected uncertainty (noise) for the blob centroid.
    
    % 对整体进行二值化处理
    frame_gray = im2gray(frame);
    fig_binarize = imbinarize(frame_gray,0.9);
    
    % 预分配内存
    numRegions = size(bbPredict, 1);
    bbox_new = cell(numRegions, 1);
    centroids_new = cell(numRegions, 1);

    % 对区域内目标进行筛选
    for i  = 1:6
        roiPosition = bbPredict(i,:);
        fig_roi = fig_binarize(roiPosition(2):roiPosition(2)+roiPosition(4)-1, roiPosition(1):roiPosition(1)+roiPosition(3)-1);
        [area,centroids,bbox,Perimeter] = detectorObjects.blobAnalysis.step(fig_roi);
        perimeterAreaRatio = (Perimeter.^2)./(double(area)*pi);
        selectedBlobs = (3.5 <perimeterAreaRatio ) & (perimeterAreaRatio  < 4.5);
        
        if ~isempty(selectedBlobs)
            bbox_new{i} = int32([bbox(selectedBlobs, 1:2) + bbPredict(i,1:2),  bbox(selectedBlobs,3:4)]);
            centroids_new{i} = centroids(selectedBlobs, :)+ double(bbPredict(i,1:2));
        else
            bbox_new{i} = int32([]);
            centroids_new{i} = [];
        end
    end
    
    centroids = cell2mat(centroids_new);
    bbox = cell2mat(bbox_new);

end

function coordinate = coordCalculate(cen_1,cen_2,stereoParams)
    
    L_new = zeros(6,2);
    

    L_new(1,:) = cen_1(5,:);
    L_new(2,:) = cen_1(6,:);
    L_new(3,:) = cen_1(4,:);
    L_new(4,:) = cen_1(3,:);
    L_new(5,:) = cen_1(2,:);
    L_new(6,:) = cen_1(1,:);
    R_new = cen_2;
    coordinate = triangulate(R_new,L_new,stereoParams);
    
end