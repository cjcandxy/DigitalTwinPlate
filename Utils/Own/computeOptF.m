function [U,nodeStrain,nodeStress,optF] = computeOptF(Elements,Nodes,K,h,E,u,conDOF,obsDof,targetLocation)

    Dof = 6;  
    ElementCount= size(Elements,1); % 单元个数
    NodeCount = size(Nodes,1) ;     % 节点个数
    
    for II = 1:ElementCount
        ElementNodeCoordinate = Nodes(Elements(II,:),1:2);
        [Bm,Bb,~] = computeBMatrix(0,0,ElementNodeCoordinate);
        % 此位置正负号和Abaqus中定义方向有关系
        B_matrix = Bm-h/2*Bb;
        fullB(II*3-2:II*3,II*24-23:II*24) = B_matrix;
    end
    
    % 将原始U阵转换为与单元对应的U阵
    tranU = zeros(ElementCount*24,NodeCount*6);
    for II = 1:ElementCount
        ElementNodeDOF = zeros(24,1);
        for J = 1:4            
            JJ=(J-1)*Dof+1;
            ElementNodeDOF(JJ:JJ+5)=(Elements(II,J)-1)*Dof+1:(Elements(II,J)-1)*Dof+6;      
        end
    
        for j = 1:24
            tranU(24*(II-1)+j,ElementNodeDOF(j)) = 1;
        end
    end
    
    % 该矩阵的功能是将高斯点应变值转换至节点应变
    % 因为所有值都是1/4乘进去的，不需磨平
    D=LinearIsotropicD(E,u);
    for I = 1:ElementCount
        tranEtoSGauss(3*(I-1)+1:3*(I-1)+3,3*(I-1)+1:3*(I-1)+3) = D(1:3,1:3);
    end
        
    tranGaussstoNode = zeros(NodeCount*3,ElementCount*3);
    for i = 1:ElementCount
        node = Elements(i,:);
        tranGaussstoNode(3*(node-1)+1,3*(i-1)+1) = 1/4;
        tranGaussstoNode(3*(node-1)+2,3*(i-1)+2) = 1/4;
        tranGaussstoNode(3*(node-1)+3,3*(i-1)+3) = 1/4;
    end
    tranUtoE = tranGaussstoNode*fullB*tranU;
      

    tranGaussstoNodeMises = zeros(NodeCount,ElementCount);
    for i = 1:ElementCount
        node = Elements(i,:);
        tranGaussstoNodeMises(node,i) = 1/4;
    end
    
    Kinv = inv(K);
    Kinvm = Kinv(:,conDOF);
    tranFtoS = tranEtoSGauss*fullB*tranU*Kinvm;
    
    tranMatrix_1 = zeros(ElementCount,ElementCount*3);
    tranMatrix_2 = zeros(ElementCount,ElementCount*3);
    for i = 1:ElementCount
        tranMatrix_1(i,3*(i-1)+1:3*(i-1)+3) = [1 1 6];
    end
    
    for i = 1:ElementCount
        tranMatrix_2(i,3*(i-1)+1:3*(i-1)+3) = [1 -1 0];
    end
    
    fun = @(x)(sqrt(0.5)*sqrt(tranMatrix_1*((tranFtoS*x).^2) + (tranMatrix_2*(tranFtoS*x)).^2));
    
    outputsize = size(obsDof,1);
    obsMatrix = zeros(outputsize,NodeCount*6);
    for i = 1:outputsize
        obsMatrix(i,obsDof(i,:)) = 1;
    end
    
    x0 = [0; 0; 0; 0; 0; 0];
    Aeq = obsMatrix*Kinvm;
    Beq = targetLocation;
    [optF,fval] = fminimax(fun,x0,[],[],Aeq,Beq);
  
    U = Kinvm*optF;

    strainGauss = fullB*tranU*U;
    stressGauss = tranEtoSGauss*strainGauss;
    nodeStrain = tranUtoE*U;
    nodeStress = tranGaussstoNode*stressGauss;
    misesWeightMatrix = [2 0 0; -2 2 0; 0 0 6];
    misesStressGauss = zeros(ElementCount,1);
    for I = 1:ElementCount
        misesStressGauss(I,1) = sqrt(0.5*stressGauss(3*(I-1)+1:3*(I-1)+3,1)'*misesWeightMatrix*stressGauss(3*(I-1)+1:3*(I-1)+3,1));
    end

end

