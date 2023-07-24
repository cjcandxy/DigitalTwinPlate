%.m Function Bk4
function  bb4=BbQ4(f,o,elementsNodesCoord)
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

x = [0,0.04];
y = [-0.04,0];

syms s t;
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

Bb1=[ 0 0 0 0 x(1)*diff(N1,s)+x(2)*diff(N1,t) 0;0 0 0 -y(1)*diff(N1,s)-y(2)*diff(N1,t) 0 0;
    0 0 0 -x(1)*diff(N1,s)-x(2)*diff(N1,t) y(1)*diff(N1,s)+y(2)*diff(N1,t) 0];
Bb2=[ 0 0 0 0 x(1)*diff(N2,s)+x(2)*diff(N2,t) 0;0 0 0 -y(1)*diff(N2,s)-y(2)*diff(N2,t) 0 0;
    0 0 0 -x(1)*diff(N2,s)-x(2)*diff(N2,t) y(1)*diff(N2,s)+y(2)*diff(N2,t) 0];
Bb3=[ 0 0 0 0 x(1)*diff(N3,s)+x(2)*diff(N3,t) 0;0 0 0 -y(1)*diff(N3,s)-y(2)*diff(N3,t) 0 0;
    0 0 0 -x(1)*diff(N3,s)-x(2)*diff(N3,t) y(1)*diff(N3,s)+y(2)*diff(N3,t) 0];
Bb4=[ 0 0 0 0 x(1)*diff(N4,s)+x(2)*diff(N4,t) 0;0 0 0 -y(1)*diff(N4,s)-y(2)*diff(N4,t) 0 0;
    0 0 0 -x(1)*diff(N4,s)-x(2)*diff(N4,t) y(1)*diff(N4,s)+y(2)*diff(N4,t) 0];
b1=[Bb1 Bb2 Bb3 Bb4];

s=f;
t=o;

b4=subs(b1);
bb4 = double(b4);
end