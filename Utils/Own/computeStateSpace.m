function [A,B,C] = computeStateSpace(M,K,ConDOF,ObsDOF,opt)

%   A: dynamics matrix
%   B: control matrix
%   C: observation Matrix
%   opt: options arguments

if isfield(opt, 'damp')
    damp_alpha = opt.damp.alpha;
    damp_belta = opt.damp.belta;
else
    damp_alpha = 0.04;
    damp_belta = 0.01;
end

dim = size(M,1);
inputsize = size(ConDOF,1);
outputsize = size(ObsDOF,1);

DD = zeros(dim,inputsize);

for i = 1:inputsize
    DD(ConDOF(i,:),i) = 1;
end

A = [zeros(dim),eye(dim);-M\K,-M\(damp_alpha*K+damp_belta*M)];
B = [zeros(dim,inputsize);M\DD];
C = zeros(outputsize,2*dim);
for i = 1:outputsize
    C(i,ObsDOF(i,:)) = 1;
end
end