function bbPredict = extendArea(bb)

bbPredict = zeros(size(bb,1),4);

bbPredict(:,1) = bb(:,1)-100;
bbPredict(:,2) = bb(:,2)-100;
bbPredict(:,3) = bb(:,3)+200;
bbPredict(:,4) = bb(:,4)+200;

bbPredict = int32(bbPredict);

end