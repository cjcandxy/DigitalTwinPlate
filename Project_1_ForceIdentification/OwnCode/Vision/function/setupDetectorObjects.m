function detectorObjects = setupDetectorObjects(area_min,area_max)
% Create System objects for foreground detection and blob analysis

detectorObjects.blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true,'AreaOutputPort', true,'CentroidOutputPort', true, ...
    'MinimumBlobArea', area_min, 'MaximumBlobArea', area_max,'MaximumCount', 50,'PerimeterOutputPort',true);

end