function [sys,x0,str,ts,simStateCompliance] = sfun_fuzzycontrol(t,x,u,flag,fuzzy_vars)
switch flag,
  % Initialization %
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  % Derivatives %
  case 1,
    sys=mdlDerivatives(t,x,u);
  % Update %
  case 2,
    sys=mdlUpdate(t,x,u);
  % Outputs %
  case 3,
    sys=mdlOutputs(t,x,u,fuzzy_vars);
  % GetTimeOfNextVarHit %
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  % Terminate %
  case 9,
    sys=mdlTerminate(t,x,u);
  % Unexpected flags %
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
% defined by the S-function parameters.
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [];
% str is always an empty matrix
str = [];
% initialize the array of sample times
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u,fuzzy_vars)
    Kpmin = fuzzy_vars(1);
    Kpmax = fuzzy_vars(2);
    Kdmin = fuzzy_vars(3);
    Kdmax = fuzzy_vars(4);
    Kimin = fuzzy_vars(5);
    Kimax = fuzzy_vars(6);
    e = u(1);
    delt_e = u(2);
    global fis
    %*********** FSG ******************
     output= evalfis(fis,[e delt_e]);
     Kpp = output(1);
     Kdp = output(2);
     Kip = output(3);
     Kp = (Kpmax - Kpmin) * Kpp + Kpmin;
     Kd = (Kdmax - Kdmin) * Kdp + Kdmin;
     Ki = (Kimax - Kimin) * Kip + Kimin;
        
sys = [Kp Ki Kd];

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];

