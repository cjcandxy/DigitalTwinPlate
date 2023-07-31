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