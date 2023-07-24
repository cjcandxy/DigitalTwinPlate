%%%%%%%%%%%  一阶六面体单元绘制云图程序  %%%%%%%%%%%
%  Nodes节点坐标信息
%  Elements单元信息
%  U位移矩阵   
%  Component云图上节点的值，可以是位移、应力、应变等
function plotNodeNumber(Nodes,Elements)


NodeCount = size(Nodes,1) ;  %  节点个数
ElementCount = size(Elements,1) ;    %单元个数
ElementNodeCount=4;  %每个单元节点数
% 矩阵初始化，X Y Z点的坐标：value点的值,对每个单元按照节点序号依次绘制云图
X = zeros(ElementNodeCount,ElementCount) ;
Y = zeros(ElementNodeCount,ElementCount) ;
Z = zeros(ElementNodeCount,ElementCount) ;
value = zeros(ElementNodeCount,ElementCount) ;
%判断矩阵类型（位移，应力，应变）

%定义colormap的颜色
myColor=1/255*[0,0,255;  0,93,255;   0,185,255;  0,255,232;
    0,255,139;  0,255,46;  46,255,0;  139,255,0;
    232,255,0;  255,185,0; 255,93,0;  255,0,0];


newNodes=Nodes;
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
    % 待优化

end
% 采用patch函数进行绘图
figure
cellfun(@patch,repmat({'Vertices'},1,ElementCount),xyz,.......%用多边形面片的方式显示各个面
    repmat({'Faces'},1,ElementCount),repmat({fm},1,ElementCount),......
    repmat({'FaceVertexCdata'},1,ElementCount),profile,......
    repmat({'FaceColor'},1,ElementCount),repmat({'interp'},1,ElementCount));
view(3);
rotate3d on;
axis off; %不显示坐标轴
axis equal;


adjustDis = max(newNodes(:,3))/10;
for i = 1:NodeCount 
    X = newNodes(i,1);
    Y = newNodes(i,2);
    Z = newNodes(i,3)+adjustDis;
    text(X,Y,Z,num2str(i),'FontSize',8);
end


end