%%%%%%%%%%%  һ�������嵥Ԫ������(���λ�ƾ���)  %%%%%%%%%%%
%  Uλ�ƾ��� 
%  E����ģ��   
%  u���ɱ�
%  Forces��������n*3 [�ڵ� ���� ��С]
%  Constraintsλ��Լ��n*3 [�ڵ� ���� ��С]
%  Nodes�ڵ�������Ϣ
%  Elements��Ԫ��Ϣ
function [U, K]=StaticsSolver(E,u,Forces,Constraints,Nodes,Elements)
Dof=3;
NodeCount = size(Nodes,1);       % �ڵ����
ElementCount= size(Elements,1);  % ��Ԫ����
Dofs = Dof*NodeCount;            % �����ɶ���
U = sparse(Dofs,1);	             % ��ʼ���ṹλ��
K = sparse(Dofs,Dofs);           % ��ʼ������ն���
Force = sparse(Dofs,1);          % ��ʼ����������
%����Ӧ��-Ӧ�����D
D=LinearIsotropicD(E,u);
for I=1:ElementCount
    % ��Ԫ�ڵ�����
    ElementNodeCoordinate=Nodes(Elements(I,:),:);
    % ���㵥��
    ElementStiffnessMatrix=Ke(D,ElementNodeCoordinate);
    % ���㵥Ԫ�ڵ����ɶȱ��
    ElementNodeDOF=zeros(1,24);
    for J=1:8
        II=(J-1)*Dof+1;
        ElementNodeDOF(II:II+2)=(Elements(I,J)-1)*Dof+1:(Elements(I,J)-1)*Dof+3;
    end
    K(ElementNodeDOF,ElementNodeDOF)=K(ElementNodeDOF,ElementNodeDOF)+ElementStiffnessMatrix;%���ݵ�Ԫ�ڵ����ɶ�  ��װ�ܸ�
end
% ʩ������
if size(Forces,1)>0
    ForceDOF = Dof*(Forces(:,1)-1)+Forces(:,2);   %�����������ɶȱ��
    Force(ForceDOF) = Force(ForceDOF) + Forces(:,3);
end
% �˴�����ʩ��λ��Լ��
% BigNumber=1e8;
% �Ҹ���
BigNumber=1e4;
ConstraintsNumber=size(Constraints,1);
if ConstraintsNumber~=0
    FixedDof=Dof*(Constraints(:,1)-1)+Constraints(:,2);  %��Լ�������ɶȱ��(������)
    for i=1:ConstraintsNumber
        K(FixedDof(i),FixedDof(i))=K(FixedDof(i),FixedDof(i))*BigNumber;
        Force(FixedDof(i))=Constraints(i,3)*K(FixedDof(i),FixedDof(i));
    end
end
%����λ��
U = K\Force;
end