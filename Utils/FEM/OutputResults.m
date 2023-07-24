%%%%%%%%%%%  һ�������嵥Ԫ����������(���txt�ļ�����ͼ)  %%%%%%%%%%%
%  OutputTXT�����Ϣ����λ��
%  Nodes�ڵ�������Ϣ 
%  Elements��Ԫ��Ϣ
%  D�������ͬ���ߵ��Բ���Ӧ��-Ӧ�����
%  Uλ�ƾ���
function OutputResults(OutputTXT,Nodes,Elements,D,U)
NodeCount = size(Nodes,1) ;  % �ڵ����
ElementCount= size(Elements,1);%��Ԫ����
ElementNodeCount=8;% ÿ����Ԫ�ڵ���
Dof=3; 
%�����˹�㡢�ڵ�Ӧ��Ӧ��ֵ
[NodeStrain,NodeStress,GaussStrain,GaussStress]=CalculateStrainAndStress(U,D,Nodes,Elements);
%���MISESӦ������
MISES=zeros(1,ElementCount*ElementNodeCount);
for I=1:ElementCount*ElementNodeCount
    MISES(I)=sqrt(0.5)*sqrt((NodeStress(1,I)-NodeStress(2,I))^2+(NodeStress(1,I)-NodeStress(3,I))^2+....
        (NodeStress(2,I)-NodeStress(3,I))^2+6*(NodeStress(4,I)^2+NodeStress(5,I)^2+NodeStress(6,I)^2));
end
%���Umagλ�ƾ���
Umag=zeros(NodeCount,1);
for i=1:NodeCount
    Umag(i)=sqrt(U(3*i-2)^2+U(3*i-1)^2+U(3*i)^2);
end
  %���Ľڵ�λ�ơ���˹��Ӧ��Ӧ��д��TXT��
  fprintf(OutputTXT,'\r\n Node          U1          U2          U3');
  for I=1:NodeCount
    II=Dof*(I-1);
    fprintf(OutputTXT,'\r\n%5d %11.3e %11.3e %11.3e',I,U(II+1:II+3)+0);
  end
  fprintf(OutputTXT,'\r\n\r\nElement GaussStrain\r\n');
  fprintf(OutputTXT,'\r\n        E11         E22         E33         E12         E23         E13');
  for I=1:ElementCount
    fprintf(OutputTXT,'\r\nElement %5d',I);
    II=(I-1)*8;
    fprintf(OutputTXT,'\r\n%11.3e %11.3e %11.3e %11.3e %11.3e %11.3e',GaussStrain(1:6,II+1:II+8));
  end
  fprintf(OutputTXT,'\r\n\r\nElement GaussStress\r\n');
  fprintf(OutputTXT,'\r\n        S11         S22         S33         S12         S23         S13');
  for I=1:ElementCount
    fprintf(OutputTXT,'\r\nElement %5d',I);
    II=(I-1)*8;
    fprintf(OutputTXT,'\r\n%11.3e %11.3e %11.3e %11.3e %11.3e %11.3e',GaussStress(1:6,II+1:II+8));
  end
  fprintf(OutputTXT,'\r\n\r\n');
  fprintf(1,'\t\t *** Successful end of program ***\n');
%���Ʊ���ǰ����
  for i=1:1:size(Elements,1)
    points=Nodes(Elements(i,:),:);
    mesh=1:1:8;%������Ϣ
    %�����嵥Ԫ�ڵ�����
    vertices_matrix = [points(mesh(1,:),1),points(mesh(1,:),2),points(mesh(1,:),3)];
    %�����嵥Ԫ�ڵ�˳��
    faces_matrix= [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];%����ÿ����Ľڵ����
    patch('vertices', vertices_matrix,'faces',faces_matrix,'facecolor','g');
    view(3);hold on%��ͼ
end
axis equal
alpha(1);
 %���λ�ơ�Ӧ����Ӧ����ͼ
for i=1:Dof %���U1-U3
    PlotContour(Nodes,Elements,U,U(i:3:size(U,1)))
    title(['U',num2str(i)]);%������ͼ����
end
for i=1:size(NodeStress,1)
    %�����Ӧ����Ӧ����ͼ S11-S33��E11-E33
    if i<4
        PlotContour(Nodes,Elements,U,NodeStrain(i,:))
        title(['E',num2str(i),num2str(i)])
        PlotContour(Nodes,Elements,U,NodeStress(i,:))
        title(['S',num2str(i),num2str(i)])
    %���S12 S23 E12 E23��ͼ
    elseif    i<6
        PlotContour(Nodes,Elements,U,NodeStrain(i,:))
        title(['E',num2str(i-3),num2str(i-2)])
        PlotContour(Nodes,Elements,U,NodeStress(i,:))
        title(['S',num2str(i-3),num2str(i-2)])
    %���S13 E13��ͼ
    else
        PlotContour(Nodes,Elements,U,NodeStrain(i,:))
        title(['E',num2str(i-5),num2str(i-3)])
        PlotContour(Nodes,Elements,U,NodeStress(i,:))
        title(['S',num2str(i-5),num2str(i-3)])
    end
end   
%����Umag��ͼ
PlotContour(Nodes,Elements,U,Umag)
title('Umag')
%����MISES��ͼ
PlotContour(Nodes,Elements,U,MISES)
title('MISES')
end