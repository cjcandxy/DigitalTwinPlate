%%%%%%%%%%%  S4R单元线弹性材料应力-应变矩阵   %%%%%%%%%%%
%  D线弹性材料应力-应变矩阵
%  E弹性模量  
%  u泊松比
function [D]=LinearIsotropicD(E,u) 
Qb = E/(1-u^2)*[1 u 0; u 1 0; 0 0 (1-u)/2];
G = E/(2*(1+u));
Qs = 5/6*G*[1 0;0 1];
D = [Qb zeros(3,2);zeros(2,3) Qs];
end