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