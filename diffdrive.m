function [sys,x0,str,ts] = diffdrive(t,x,u,flag)


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(5,1);
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
%% Place system here 
r=2;
d=4;
ke1 = 0.01; 
ke2 = 0.01;
jeq1 = 0.01;
jeq2 = 0.012;
beq1 = 0.1;
beq2 = 0.11;
kt1 = 0.3;
kt2 = 0.35;
Ra1 = 1;
Ra2 = 1.1;

%change of variables
x1=x(1); %x
x2=x(2); %y
phi=x(3); %phi
uL=x(4); %uL
uR=x(5); %uR
VL=u(1); %VL
VR=u(2); %VR

%system states sys is your state derivatives
sys(1)=(r/2)*(uR+uL)*cos(phi);
sys(2)=(r/2)*(uR+uL)*sin(phi);
sys(3)=(r/(2*d))*(uR-uL);
sys(4)=(((kt1/((beq1*Ra1)+(kt1*ke1)))*VL)-uL)/((jeq1*Ra1)/((beq1*Ra1)+(kt1*ke1)));
sys(5)=(((kt2/((beq2*Ra2)+(kt2*ke2)))*VR)-uR)/((jeq2*Ra2)/((beq2*Ra2)+(kt2*ke2)));

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
C=eye(5);
sys = C*x;

% end mdlOutputs
