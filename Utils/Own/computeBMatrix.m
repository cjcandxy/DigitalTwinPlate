function  [Bm, Bb, Bs] = computeBMatrix(s,t,elementsNodesCoord)

x1 = elementsNodesCoord(1,1);
y1 = elementsNodesCoord(1,2);
x2 = elementsNodesCoord(2,1);
y2 = elementsNodesCoord(2,2);
x3 = elementsNodesCoord(3,1);
y3 = elementsNodesCoord(3,2);
x4 = elementsNodesCoord(4,1);
y4 = elementsNodesCoord(4,2);

N1 = (1-s)*(1-t)/4;
N2 = (1+s)*(1-t)/4;
N3 = (1+s)*(1+t)/4;
N4 = (1-s)*(1+t)/4;
% N5 = (1-(s)^2)*(1-t)/16;
% N6 = (1+s)*(1-(t)^2)/16;
% N7 = (1-(s)^2)*(1+t)/16;
% N8 = (1-s)*(1-(t)^2)/16;
% L1 = (y1-y4)*N8-(y2-y1)*N5;
% L2 = (y2-y1)*N5-(y3-y2)*N6;
% L3 = (y3-y2)*N6-(y4-y3)*N7;
% L4 = (y4-y3)*N7-(y1-y4)*N8;
% M1 = (x4-x1)*N8-(x1-x2)*N5;
% M2 = (x1-x2)*N5-(x2-x3)*N6;
% M3 = (x2-x3)*N6-(x3-x4)*N7;
% M4 = (x3-x4)*N7-(x4-x1)*N8;

N1s = t/4 - 1/4;
N2s = 1/4 - t/4;
N3s = t/4 + 1/4;
N4s = -t/4 - 1/4;

N1t = s/4 - 1/4;
N2t = -s/4 - 1/4;
N3t = s/4 + 1/4;
N4t = 1/4 - s/4;

L1s = ((t^2 - 1)*(y1 - y4))/16 + (s*(y1 - y2)*(t - 1))/8;
L2s = - ((t^2 - 1)*(y2 - y3))/16 - (s*(y1 - y2)*(t - 1))/8;
L3s = ((t^2 - 1)*(y2 - y3))/16 - (s*(y3 - y4)*(t + 1))/8;
L4s = (s*(y3 - y4)*(t + 1))/8 - ((t^2 - 1)*(y1 - y4))/16;

L1t = ((s^2 - 1)*(y1 - y2))/16 + (t*(y1 - y4)*(s - 1))/8;
L2t = - ((s^2 - 1)*(y1 - y2))/16 - (t*(y2 - y3)*(s + 1))/8;
L3t = (t*(y2 - y3)*(s + 1))/8 - ((s^2 - 1)*(y3 - y4))/16;
L4t = ((s^2 - 1)*(y3 - y4))/16 - (t*(y1 - y4)*(s - 1))/8;

M1s =  - ((t^2 - 1)*(x1 - x4))/16 - (s*(x1 - x2)*(t - 1))/8;
M2s = ((t^2 - 1)*(x2 - x3))/16 + (s*(x1 - x2)*(t - 1))/8;
M3s = (s*(x3 - x4)*(t + 1))/8 - ((t^2 - 1)*(x2 - x3))/16;
M4s = ((t^2 - 1)*(x1 - x4))/16 - (s*(x3 - x4)*(t + 1))/8;

M1t =  - ((s^2 - 1)*(x1 - x2))/16 - (t*(x1 - x4)*(s - 1))/8;
M2t = ((s^2 - 1)*(x1 - x2))/16 + (t*(x2 - x3)*(s + 1))/8;
M3t = ((s^2 - 1)*(x3 - x4))/16 - (t*(x2 - x3)*(s + 1))/8;
M4t = (t*(x1 - x4)*(s - 1))/8 - ((s^2 - 1)*(x3 - x4))/16;

J = [N1s N2s N3s N4s;N1t N2t N3t N4t]*elementsNodesCoord;

jacobi = inv(J);

x = jacobi(1,:);
y = jacobi(2,:);

Bm1 = [x(1)*N1s+x(2)*N1t 0 0 0 0 x(1)*L1s+x(2)*L1t;0 y(1)*N1s+y(2)*N1t 0 0 0 x(1)*M1s+x(2)*M1t;
    y(1)*N1s+y(2)*N1t x(1)*N1s+x(2)*N1t 0 0 0 y(1)*L1s+y(2)*L1t+x(1)*M1s+x(2)*M1t];
Bm2 = [x(1)*N2s+x(2)*N2t 0 0 0 0 x(1)*L2s+x(2)*L2t;0 y(1)*N2s+y(2)*N2t 0 0 0 x(1)*M2s+x(2)*M2t;
    y(1)*N2s+y(2)*N2t x(1)*N2s+x(2)*N2t 0 0 0 y(1)*L2s+y(2)*L2t+x(1)*M2s+x(2)*M2t];
Bm3 = [x(1)*N3s+x(2)*N3t 0 0 0 0 x(1)*L3s+x(2)*L3t;0 y(1)*N3s+y(2)*N3t 0 0 0 x(1)*M3s+x(2)*M3t;
    y(1)*N3s+y(2)*N3t x(1)*N3s+x(2)*N3t 0 0 0 y(1)*L3s+y(2)*L3t+x(1)*M3s+x(2)*M3t];
Bm4 = [x(1)*N4s+x(2)*N4t 0 0 0 0 x(1)*L4s+x(2)*L4t;0 y(1)*N4s+y(2)*N4t 0 0 0 x(1)*M4s+x(2)*M4t;
    y(1)*N4s+y(2)*N4t x(1)*N4s+x(2)*N4t 0 0 0 y(1)*L4s+y(2)*L4t+x(1)*M4s+x(2)*M4t];
Bm = [Bm1 Bm2 Bm3 Bm4];

Bb1 = [ 0 0 0 0 x(1)*N1s+x(2)*N1t 0;0 0 0 -y(1)*N1s-y(2)*N1t 0 0;
    0 0 0 -x(1)*N1s-x(2)*N1t y(1)*N1s+y(2)*N1t 0];
Bb2 = [ 0 0 0 0 x(1)*N2s+x(2)*N2t 0;0 0 0 -y(1)*N2s-y(2)*N2t 0 0;
    0 0 0 -x(1)*N2s-x(2)*N2t y(1)*N2s+y(2)*N2t 0];
Bb3 = [ 0 0 0 0 x(1)*N3s+x(2)*N3t 0;0 0 0 -y(1)*N3s-y(2)*N3t 0 0;
    0 0 0 -x(1)*N3s-x(2)*N3t y(1)*N3s+y(2)*N3t 0];
Bb4 = [ 0 0 0 0 x(1)*N4s+x(2)*N4t 0;0 0 0 -y(1)*N4s-y(2)*N4t 0 0;
    0 0 0 -x(1)*N4s-x(2)*N4t y(1)*N4s+y(2)*N4t 0];
Bb = [Bb1 Bb2 Bb3 Bb4];

Bs1 = [ 0 0 x(1)*N1s+x(2)*N1t -(x(1)*L1s+x(2)*L1t) -(x(1)*M1s+x(2)*M1t)+N1 0;
    0 0 y(1)*N1s+y(2)*N1t -(y(1)*L1s+y(2)*L1t)-N1 -(y(1)*M1s+y(2)*M1t) 0];
Bs2 = [ 0 0 x(1)*N2s+x(2)*N2t -(x(1)*L2s+x(2)*L2t) -(x(1)*M2s+x(2)*M2t)+N2 0;
    0 0 y(1)*N2s+y(2)*N2t -(y(1)*L2s+y(2)*L2t)-N2 -(y(1)*M1s+y(2)*M1t) 0];
Bs3 = [ 0 0 x(1)*N3s+x(2)*N3t -(x(1)*L3s+x(2)*L3t) -(x(1)*M3s+x(2)*M3t)+N3 0;
    0 0 y(1)*N3s+y(2)*N3t -(y(1)*L3s+y(2)*L3t)-N3 -(y(1)*M1s+y(2)*M1t) 0];
Bs4 = [ 0 0 x(1)*N4s+x(2)*N4t -(x(1)*L4s+x(2)*L4t) -(x(1)*M4s+x(2)*M4t)+N4 0;
    0 0 y(1)*N4s+y(2)*N4t -(y(1)*L4s+y(2)*L4t)-N4 -(y(1)*M1s+y(2)*M1t) 0];
Bs = [Bs1 Bs2 Bs3 Bs4];

end