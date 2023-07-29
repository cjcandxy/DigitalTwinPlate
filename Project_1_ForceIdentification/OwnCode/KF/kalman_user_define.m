function [kest,L,P,Mx,Z,My] = kalman_user_define(sys,Qn,Rn,varargin)
%KALMAN  Kalman state estimator.
%
%   [KEST,L,P] = KALMAN(SYS,QN,RN,NN) designs a Kalman estimator KEST for
%   the continuous- or discrete-time plant SYS. For continuous-time plants
%      .
%      x = Ax + Bu + Gw            {State equation}
%      y = Cx + Du + Hw + v        {Measurements}
%
%   with known inputs u, process disturbances w, and measurement noise v,
%   KEST uses [u(t);y(t)] to generate optimal estimates y_e(t),x_e(t) of 
%   y(t),x(t) by:
%       .
%      x_e  = Ax_e + Bu + L (y - Cx_e - Du)
%
%      |y_e| = | C | x_e + | D | u
%      |x_e|   | I |       | 0 | 
%
%   KALMAN takes the state-space model SYS=SS(A,[B G],C,[D H]) and the 
%   covariance matrices:
%
%      QN = E{ww'},     RN = E{vv'},     NN = E{wv'}.
%
%   The row size of QN specifies the length of w and NN is set to 0 when 
%   omitted. KALMAN returns the estimator gain L and the steady-state error 
%   covariance P (solution of the associated Riccati equation).
%
%   [KEST,L,P] = KALMAN(SYS,QN,RN,NN,SENSORS,KNOWN) handles more general 
%   situations where
%      * Not all outputs of SYS are measured
%      * The disturbance inputs w are not the last inputs of SYS.
%   The index vectors SENSORS and KNOWN then specify which outputs y of SYS
%   are measured and which inputs u to SYS are known. All other inputs of
%   SYS are assumed stochastic.
%
%   For discrete-time plants, KALMAN can compute a "current" or "delayed"
%   Kalman estimator. The "current" estimator uses all measurements up to
%   y[n] to estimate x[n]. The "delayed" estimator uses only past 
%   measurements up to y[n-1] and is easier to embed in digital control 
%   loops. The current estimator has equations:
%
%      x[n+1|n] = Ax[n|n-1] + Bu[n] + L (y[n] - Cx[n|n-1] - Du[n])
%      y[n|n]  =  Cx[n|n-1] + Du[n] + My (y[n] - Cx[n|n-1] - Du[n])
%      x[n|n]  =  x[n|n-1] + Mx (y[n] - Cx[n|n-1] - Du[n])
%    
%   Note that My=C*Mx and y[n|n]=Cx[n|n]+Du[n] when H=0. The delayed 
%   estimator has the same state equation but outputs x[n|n-1] and
%   y[n|n-1] = Cx[n|n-1]+Du[n] rather than x[n|n] and y[n|n].
%
%   [KEST,L,P,Mx,Z,My] = KALMAN(SYS,QN,RN,...,TYPE) specifies the estimator
%   type for discrete-time plants SYS. The string TYPE is either 'current'
%   (default) or 'delayed'. KALMAN returns the estimator and innovation 
%   gains L,Mx,My and the steady-state error covariances:
%
%       P = E{(x - x[n|n-1])(x - x[n|n-1])'}   (Riccati solution)
%       Z = E{(x - x[n|n])(x - x[n|n])'}
%
%   See also KALMD, ESTIM, LQGREG, LQG, SS, ICARE, IDARE.

%   Author(s): P. Gahinet
%   Copyright 1986-2018 The MathWorks, Inc.
narginchk(3,7)
if ndims(sys)>2 %#ok<ISMAT>
   error(message('Control:general:RequiresSingleModel','kalman'))
elseif hasdelay(sys)
   throw(ltipack.utNoDelaySupport('kalman',sys.Ts,'all'))
end
ni = nargin;

% Look for 'current', 'delayed', or 'lqg' flag
ix = find(cellfun(@(s) any(strcmpi(s,["current";"delayed";"lqg"])),varargin));
if isempty(ix)
   DiscreteType = 'current'; % y[n|n], x[n|n] are the filter's output
else
   DiscreteType = varargin{ix};   varargin(:,ix) = [];  ni = ni-1;
end

% Extract plant data
[A,BB,C,DD,~,Ts] = dssdata(sys);
E = sys.E;
Nx = size(A,1);
[pd,md] = size(DD);

% Eliminate outputs that are not sensors
if ni<5
   sensors = 1:pd;
else
   sensors = varargin{2};
   if any(sensors<=0) || any(sensors>pd)
      error(message('Control:general:IndexOutOfRange','kalman(SYS,QN,RN,NN,SENSORS,...)','SENSORS'))
   end
end
C = C(sensors,:);  DD = DD(sensors,:);
Ny = size(DD,1); % number of measurements y
Nw = size(Qn,1); % number of disturbances w
if Nw>md
   error(message('Control:design:kalman2',md))
end

% Validate Qn,Rn,Nn
if ni<4
   Nn = zeros(Nw,Ny);
else
   Nn = varargin{1};
end
try
   [Qn,Rn,Nn] = ltipack.checkQRS(Nw,Ny,Qn,Rn,Nn,{'QN','RN','NN'});
catch ME
   throw(ME)
end

% Extract B,G,D,H
Nu = md-Nw;
if ni<6
   % Stochastic inputs w are the last Nw=SIZE(Qn,2) inputs
   % and known inputs u are the remaining ones.
   known = 1:Nu;
else
   known = varargin{3};
   if any(known<=0) || any(known>md)
      error(message('Control:general:IndexOutOfRange','kalman(SYS,QN,RN,NN,SENSORS,KNOWN)','KNOWN'))
   elseif length(known)~=Nu
      error(message('Control:design:kalman5'))
   end
end
stoch = 1:md;
stoch(known) = [];
B = BB(:,known);  D = DD(:,known);
G = BB(:,stoch);  H = DD(:,stoch);

% Factor [Qn Nn;Nn' Rn] and use square-root formulation when possible
[F1,F2,INDEF] = ltipack.factorQRS(Qn,Rn,Nn);
if INDEF
   % Explicitly form aggregate covariance matrices
   %
   %  [ Qb  Nb ]     [ G  0 ] [ Qn  Nn ] [ G  0 ]'
   %  [        ]  =  [      ] [        ] [      ]
   %  [ Nb' Rb ]     [ H  I ] [ Nn  Rn ] [ H  I ]
   %
   aux1 = H*Qn+Nn';  aux2 = H*Nn+Rn;
   Q = G * Qn * G';  Q = (Q+Q')/2;
   R = aux1 * H' + aux2;  R = (R+R')/2;
   
   N = G * aux1';
   CC = C;
   % Warn if numerically indefinite
   ev = eig([Q N;N' R]);
   if min(ev)<-1e2*eps*max(abs(ev))
      warning(message('Control:design:MustBePositiveDefinite','[G 0;H I]*[Qn Nn;Nn'' Rn]*[G 0;H I]''','kalman'))
   end
else
   % Use factored form of [Qb Nb;Nb' Rb] to avoid squaring up
   CC = [C ; zeros(Nw+Ny,Nx)];
   Q = zeros(Nx);
   N = [zeros(Nx,Ny) , G*F1];
   F2 = F2 + H*F1;
   R = [zeros(Ny) F2;F2' -eye(Nw+Ny)];
end

% Solve Riccati equation
if Ts==0
   [P,K,~,INFO] = icare(A',CC',Q,R,N,E');
else
   [P,K,~,INFO] = idare(A',CC',Q,R,N,E');
end

% Handle failures
switch INFO.Report
   case 2
      % P and L are not finite
      error(message('Control:design:kalman3'))
   case 3
      % Could not compute stabilizing P
      error(message('Control:design:kalman4'))
end

% Build Kalman estimator (Ae,Be,Ce,De)
L = K(1:Ny,:)';
Ae = A-L*C;
Be = [B-L*D , L];
Mx = [];  My = [];  Z = [];  wEstimates = [];
if Ts==0
   % Continuous state estimator
   %      .                             |u|
   %     x_e  = [A-LC] x_e + [B-LD , L] |y|
   %
   %    |y_e| = [C] x_e + [D 0] |u|
   %    |x_e| = [I]       [0 0] |y|
   Ce = [C ; eye(Nx)];
   De = [D zeros(Ny);zeros(Nx,Nu+Ny)];
else
   % Discrete state estimator:
   %                                             |u[n]|
   %    x[n+1|n] = [A-LC] x[n|n-1] + [B-LD , L]  |y[n]|
   %
   %    L = (APC'+Nb) / (CPC'+Rb) ,  P = solution of DARE
   
   % Without forming CPC'+Rb, compute 
   %    Mx = PC'/(CPC'+Rb)
   %    Mw = (Qn*H'+Nn)/(CPC'+Rb)
   U = INFO.U;  V = INFO.V;  Sx = INFO.Sx;
   if isempty(E)
      PSI = [-(Sx.\U)' , V'*(Sx.*CC') ; CC , R];
   else
      PSI = [-(Sx.\U)'*E , V'*(Sx.*CC') ; CC , R];
   end
   m = size(R,1)-Ny;
   aux = [eye(Nx) zeros(Nx,Ny+m) ; zeros(Nw,Nx) Qn*H'+Nn zeros(Nw,m)]/PSI;
   Mx = aux(1:Nx,Nx+1:Nx+Ny);
   Mw = aux(Nx+1:Nx+Nw,Nx+1:Nx+Ny);
   
   % Compute Z = (I-Mx*C)*P
   Z = P-Mx*(C*P);
   Z = (Z+Z')/2;
   
   % Compute estimator matrices
   switch lower(DiscreteType)
      case 'delayed'
         % "delayed" estimator
         %    |y[n|n-1]| = [   C   ] x[n|n-1] + [D   0]  |u[n]|
         %    |x[n|n-1]| = [   I   ]            [0   0]  |y[n]|
         Ce = [C ; eye(Nx)];
         De = [D zeros(Ny);zeros(Nx,Nu+Ny)];
      case 'current'
         % "current" estimator
         %    |y[n|n]| = [ (I-My)*C ] x[n|n-1] + [(I-My)*D  My]  |u[n]|
         %    |x[n|n]| = [  I-Mx*C  ]            [ -Mx*D    Mx]  |y[n]|
         My = C*Mx+H*Mw;       % My = (CPC'+HQH'+HN)/(CPC'+Rb)
         imy = eye(Ny)-My;
         Ce = [imy*C ; eye(Nx)-Mx*C];
         De = [imy*D My; -Mx*D Mx];
     case 'lqg'
         % Estimator for LQG-optimal controller (see LQG)
         %    |w[n|n]| = [   -Mw*C  ] x[n|n-1] + [ -Mw*D    Mw]  |u[n]|
         %    |x[n|n]| = [  I-Mx*C  ]            [ -Mx*D    Mx]  |y[n]|
         % Note that 
         %   * assumes G=I (absorbed into Qn)
         %   * y[n|n] = C x[n|n] + D u[n] + H w[n|n]
         Ce = [-Mw*C ; eye(Nx)-Mx*C];
         De = [-Mw*D Mw; -Mx*D Mx];
         My = Mw;   % return Mw as last output
         wEstimates = stoch;
   end
end

% Build estimator
kest = ss(Ae,Be,Ce,De,Ts,'E',E,'TimeUnit',sys.TimeUnit);

% Set metadata
% kest = estimMetaData(kest,sys,known,sensors,wEstimates);
