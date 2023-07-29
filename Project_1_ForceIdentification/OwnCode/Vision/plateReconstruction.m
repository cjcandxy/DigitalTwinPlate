%% 加载初始化数据
clear variables
clc

% 导入先验信息
% romMatrix    - 导入降阶模型矩阵
    % 降阶结果中应包含视觉坐标系到全局坐标系信息
% visionInit   - 定义视觉初始化矩阵
    % stereoMatrix - 双目相机标定矩阵
    % boxInit        - 定义初始目标检测框（拓展）位置
    % coordinateInit - 定义初始坐标位置
    % matchInit      - 定义初始化检测信息

load romMatrix.mat
load visionInit.mat

%% 初始化标志点跟踪及重建系统

% 初始化检测系统对象
area_min = 300; area_max = 1000;
detectorObjects = setupDetectorObjects(area_min,area_max);

% 绘制初始图像
U_int = zeros(1080,1);
h = patch('Vertices',Nodes,'Faces',Elements,'FaceVertexCData',U_int,'FaceColor','interp','EdgeAlpha',0);
view(3);

clim([0,60]);
t1=clim;
t1=linspace(t1(1),t1(2),13);
colorbar('ytick',t1,'Location','westoutside');
colormap(jet);
axis equal
axis off

%% 初始化跟踪模型
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












