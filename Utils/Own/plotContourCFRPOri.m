%%%%%%%%%%%  一阶六面体单元绘制云图程序  %%%%%%%%%%%
%  Nodes节点坐标信息
%  Elements单元信息
%  U位移矩阵   
%  Component云图上节点的值，可以是位移、应力、应变等
function plotContourCFRPOri(Nodes,Elements,U,Component,scalefactor,minvalue,maxvalue)
NodeCount = size(Nodes,1) ;  %  节点个数
ElementCount = size(Elements,1) ;    %单元个数
ElementNodeCount=8;  %每个单元节点数
% 矩阵初始化，X Y Z点的坐标：value点的值,对每个单元按照节点序号依次绘制云图
X = zeros(ElementNodeCount,ElementCount) ;
Y = zeros(ElementNodeCount,ElementCount) ;
Z = zeros(ElementNodeCount,ElementCount) ;
value = zeros(ElementNodeCount,ElementCount) ;
%判断矩阵类型（位移，应力，应变）
if size(Component,1)>1
    for i=1:ElementCount
        nd=Elements(i,:);
        value(:,i) = Component(nd) ;
    end
else
    %先进行磨平，再把Component行向量转化成矩阵形式
    Difference=max(Component)-min(Component);%全域上的最大值-最小值
    AVG=0.75;  % 默认阈值75%
    for i=1:1:NodeCount %遍历节点，进行应力磨平
        TElements=Elements';%转置Elements
        itemp=(TElements==i);%进行逻辑判断，itemp:元素为0 1的矩阵
        Cut=max(Component(1,itemp))-min(Component(1,itemp));%该节点的应力（应变）差
        if 0<Cut&&Cut<=AVG*Difference(1)%判断是否满足阈值条件
            Component(1,itemp)=mean(Component(1,itemp));%进行应力（应变）磨平
        end
    end
    value=reshape(Component,ElementNodeCount,ElementCount);%将Component的值赋给value
end



%定义colormap的颜色
% myColor=1/255*[0,0,255;  0,93,255;   0,185,255;  0,255,232;
%     0,255,139;  0,255,46;  46,255,0;  139,255,0;
%     232,255,0;  255,185,0; 255,93,0;  255,0,0];
myColor = jet;
% 绘制变形后云图
U1 = zeros(size(U,1),1);
U1(1:3:end) = U(1:3:end);
U1(2:3:end) = U(2:3:end);
U1(3:3:end) = U(3:3:end);

newNodes=Nodes';
newNodes=newNodes(:);

DeformationCoefficient=scalefactor; %变形放大系数
newNodes=newNodes+DeformationCoefficient*U1;

newNodes=reshape(newNodes,[3,size(Nodes,1)]);
newNodes=newNodes';

fm = [1 2 3 4]; 

xyz = cell(1,ElementCount) ;
profile = xyz ;
%将节点位置赋予其对应值
for e=1:ElementCount %循环获取每个单元内节点的坐标
    nd=Elements(e,:);
    X = newNodes(nd,1) ;
    Y = newNodes(nd,2) ;
    Z = newNodes(nd,3) ;
    xyz{e} = [X  Y Z] ;
    profile{e} = value(:,e);
end
% 采用patch函数进行绘图
% figure
% edge color none



cellfun(@patch,repmat({'Vertices'},1,ElementCount),xyz,.......%用多边形面片的方式显示各个面
    repmat({'Faces'},1,ElementCount),repmat({fm},1,ElementCount),......
    repmat({'FaceVertexCdata'},1,ElementCount),profile,......
    repmat({'FaceColor'},1,ElementCount),repmat({'black'},1,ElementCount), ...
    repmat({'EdgeColor'},1,ElementCount),repmat({'none'},1,ElementCount), ...
    repmat({'FaceAlpha'},1,ElementCount),repmat({'0.3'},1,ElementCount));

% cellfun(@patch,repmat({'Vertices'},1,ElementCount),xyz,.......%用多边形面片的方式显示各个面
%     repmat({'Faces'},1,ElementCount),repmat({fm},1,ElementCount),......
%     repmat({'FaceVertexCdata'},1,ElementCount),profile,......
%     repmat({'FaceColor'},1,ElementCount),repmat({'interp'},1,ElementCount));
% 
% view(3);
rotate3d on;
axis off; %不显示坐标轴
colormap(myColor);
% clim([minvalue,maxvalue]);
% t1=clim;
% t1=linspace(t1(1),t1(2),13);
% colorbar('ytick',t1,'Location','westoutside');
axis equal;
end