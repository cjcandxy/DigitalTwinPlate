%%%%%%%%%%%  һ�������嵥Ԫ������ͼ����  %%%%%%%%%%%
%  Nodes�ڵ�������Ϣ
%  Elements��Ԫ��Ϣ
%  Uλ�ƾ���   
%  Component��ͼ�Ͻڵ��ֵ��������λ�ơ�Ӧ����Ӧ���
function plotContourCFRP(Nodes,Elements,U,Component,scalefactor,minvalue,maxvalue)
NodeCount = size(Nodes,1) ;  %  �ڵ����
ElementCount = size(Elements,1) ;    %��Ԫ����
ElementNodeCount=8;  %ÿ����Ԫ�ڵ���
% �����ʼ����X Y Z������꣺value���ֵ,��ÿ����Ԫ���սڵ�������λ�����ͼ
X = zeros(ElementNodeCount,ElementCount) ;
Y = zeros(ElementNodeCount,ElementCount) ;
Z = zeros(ElementNodeCount,ElementCount) ;
value = zeros(ElementNodeCount,ElementCount) ;
%�жϾ������ͣ�λ�ƣ�Ӧ����Ӧ�䣩
if size(Component,1)>1
    for i=1:ElementCount
        nd=Elements(i,:);
        value(:,i) = Component(nd) ;
    end
else
    %�Ƚ���ĥƽ���ٰ�Component������ת���ɾ�����ʽ
    Difference=max(Component)-min(Component);%ȫ���ϵ����ֵ-��Сֵ
    AVG=0.75;  % Ĭ����ֵ75%
    for i=1:1:NodeCount %�����ڵ㣬����Ӧ��ĥƽ
        TElements=Elements';%ת��Elements
        itemp=(TElements==i);%�����߼��жϣ�itemp:Ԫ��Ϊ0 1�ľ���
        Cut=max(Component(1,itemp))-min(Component(1,itemp));%�ýڵ��Ӧ����Ӧ�䣩��
        if 0<Cut&&Cut<=AVG*Difference(1)%�ж��Ƿ�������ֵ����
            Component(1,itemp)=mean(Component(1,itemp));%����Ӧ����Ӧ�䣩ĥƽ
        end
    end
    value=reshape(Component,ElementNodeCount,ElementCount);%��Component��ֵ����value
end



%����colormap����ɫ
% myColor=1/255*[0,0,255;  0,93,255;   0,185,255;  0,255,232;
%     0,255,139;  0,255,46;  46,255,0;  139,255,0;
%     232,255,0;  255,185,0; 255,93,0;  255,0,0];
myColor = jet;
% ���Ʊ��κ���ͼ
U1 = zeros(size(U,1),1);
U1(1:3:end) = U(1:3:end);
U1(2:3:end) = U(2:3:end);
U1(3:3:end) = U(3:3:end);

newNodes=Nodes';
newNodes=newNodes(:);

DeformationCoefficient=scalefactor; %���ηŴ�ϵ��
newNodes=newNodes+DeformationCoefficient*U1;

newNodes=reshape(newNodes,[3,size(Nodes,1)]);
newNodes=newNodes';

fm = [1 2 3 4]; 

xyz = cell(1,ElementCount) ;
profile = xyz ;
%���ڵ�λ�ø������Ӧֵ
for e=1:ElementCount %ѭ����ȡÿ����Ԫ�ڽڵ������
    nd=Elements(e,:);
    X = newNodes(nd,1) ;
    Y = newNodes(nd,2) ;
    Z = newNodes(nd,3) ;
    xyz{e} = [X  Y Z] ;
    profile{e} = value(:,e);
end
% ����patch�������л�ͼ
% figure
% edge color none


cellfun(@patch,repmat({'Vertices'},1,ElementCount),xyz,.......%�ö������Ƭ�ķ�ʽ��ʾ������
    repmat({'Faces'},1,ElementCount),repmat({fm},1,ElementCount),......
    repmat({'FaceVertexCdata'},1,ElementCount),profile,......
    repmat({'FaceColor'},1,ElementCount),repmat({'interp'},1,ElementCount), ...
    repmat({'EdgeColor'},1,ElementCount),repmat({'none'},1,ElementCount));
% cellfun(@patch,repmat({'Vertices'},1,ElementCount),xyz,.......%�ö������Ƭ�ķ�ʽ��ʾ������
%     repmat({'Faces'},1,ElementCount),repmat({fm},1,ElementCount),......
%     repmat({'FaceVertexCdata'},1,ElementCount),profile,......
%     repmat({'FaceColor'},1,ElementCount),repmat({'interp'},1,ElementCount));

% view(3);
rotate3d on;
axis off; %����ʾ������
colormap(myColor);
% clim([minvalue,maxvalue]);
% t1=clim;
% t1=linspace(t1(1),t1(2),13);
% colorbar('ytick',t1,'Location','westoutside');
axis equal;
end