%Shpe function,only related to element location
%.m Function Bs
function  bs4=BsQ4(f,o,elementsNodesCoord)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

x1 = elementsNodesCoord(1,1);
y1 = elementsNodesCoord(1,2);
x2 = elementsNodesCoord(2,1);
y2 = elementsNodesCoord(2,2);
x3 = elementsNodesCoord(3,1);
y3 = elementsNodesCoord(3,2);
x4 = elementsNodesCoord(4,1);
y4 = elementsNodesCoord(4,2);

x = 0.04;
y = 0.04;

syms s t;    %%s,t : Axis of irregular units

N1=(1-s)*(1-t)/4;
N2=(1+s)*(1-t)/4;
N3=(1+s)*(1+t)/4;
N4=(1-s)*(1+t)/4;
N5=(1-(s)^2)*(1-t)/16;
N6=(1+s)*(1-(t)^2)/16;
N7=(1-(s)^2)*(1+t)/16;
N8=(1-s)*(1-(t)^2)/16;
L1=(y1-y4)*N8-(y2-y1)*N5;
L2=(y2-y1)*N5-(y3-y2)*N6;
L3=(y3-y2)*N6-(y4-y3)*N7;
L4=(y4-y3)*N7-(y1-y4)*N8;
M1=(x4-x1)*N8-(x1-x2)*N5;
M2=(x1-x2)*N5-(x2-x3)*N6;
M3=(x2-x3)*N6-(x3-x4)*N7;
M4=(x3-x4)*N7-(x4-x1)*N8;

Bs1=[ 0 0 x*diff(N1,s) -x*diff(L1,s) -x*diff(M1,s)+N1 0;0 0 y*diff(N1,t) -y*diff(L1,t)-N1 -y*diff(M1,t) 0];
Bs2=[ 0 0 x*diff(N2,s) -x*diff(L2,s) -x*diff(M2,s)+N2 0;0 0 y*diff(N2,t) -y*diff(L2,t)-N2 -y*diff(M2,t) 0];
Bs3=[ 0 0 x*diff(N3,s) -x*diff(L3,s) -x*diff(M3,s)+N3 0;0 0 y*diff(N3,t) -y*diff(L3,t)-N3 -y*diff(M3,t) 0];
Bs4=[ 0 0 x*diff(N4,s) -x*diff(L4,s) -x*diff(M4,s)+N4 0;0 0 y*diff(N4,t) -y*diff(L4,t)-N4 -y*diff(M4,t) 0];
s1=[Bs1 Bs2 Bs3 Bs4];
s=f;t=o;
s4=subs(s1);
bs4 = double(s4);
end