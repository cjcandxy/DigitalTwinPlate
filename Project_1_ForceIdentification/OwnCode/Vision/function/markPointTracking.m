function [trackers,bbPredict,cen] = markPointTracking(frameCount,thisFrameCentroids,thisFrameBboxes,tracker)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    numMeasurementsInFrame = size(thisFrameCentroids,1);
    % 用于检测的cvekf tracker x,y,z输入，对于图片，默认z为0
    zeroColumn = zeros(numMeasurementsInFrame, 1);

    thisFrameMeasure = [thisFrameCentroids, zeroColumn];

    detectionsInFrame = cell(numMeasurementsInFrame, 1);

    for detCount = 1:numMeasurementsInFrame
        detectionsInFrame{detCount} = objectDetection(...
            frameCount, ... % Use frame count as time
            thisFrameMeasure(detCount,:), ... % Use centroid as measurement in pixels
            MeasurementNoise = diag([10 10 0]), ... % Centroid measurement noise in pixels
            ObjectAttributes = struct(BoundingBox = thisFrameBboxes(detCount,:)) ... % Attach bounding box information
            );
    end    
    
    detectionHistory{frameCount} = detectionsInFrame;

    if isLocked(tracker) || ~isempty(detectionHistory{frameCount})
        trackers = tracker(detectionHistory{frameCount}, frameCount);
    else
        trackers = objectTrack.empty;
    end
    
    % insertInformation
    numTracks = numel(trackers);
    boxes = zeros(numTracks, 4);
    cen = zeros(numTracks, 2);
    ids = zeros(numTracks, 1, "int32");
    predictedTrackInds = zeros(numTracks, 1);

    for tr = 1:numTracks
        % Get bounding boxes.
        boxes(tr, :) = trackers(tr).ObjectAttributes.BoundingBox;
        boxes(tr, 1:2) = (trackers(tr).State(1:2:3))'-boxes(tr,3:4)/2;
        cen(tr,:) = (trackers(tr).State(1:2:3))';
        % Get IDs.
        ids(tr) = trackers(tr).TrackID;
        
        % 表示目标已经超出了范围，并且没有被跟踪到
        if trackers(tr).IsCoasted
            predictedTrackInds(tr) = tr;
        end
    end
       
    bbPredict = extendArea(boxes);
       
end