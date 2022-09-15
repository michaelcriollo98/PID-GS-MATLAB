function [sys,x0,str,ts,simStateCompliance] = sfun_SA(t,x,u,flag,SA_vars)
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
    sys=mdlOutputs(t,x,u,SA_vars);
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
sizes.NumOutputs     = 4;
sizes.NumInputs      = 9;
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

function sys=mdlOutputs(t,x,u,SA_vars)
global SA 

 alpha = SA_vars(1);
  Temp = SA_vars(2);
select = SA_vars(3);
    Ts = SA_vars(4);
range_gains = SA_vars(5:end);
    
       e = u(1);
     ref = u(2);
  enable = u(3); 
    iter = u(4);
 subiter = u(5);
sol.Cost = u(6);
sol.Gain = u(7:9);

y = ref - e;

if enable == 1 && length(SA(select).y) > 1
     Temp = Temp * (alpha)^(iter-1);
    if subiter == 0 && iter == 0 
        sol.Cost = CostFunction(SA(select).y,ref,Ts,SA(select).selectCostF);
        sol.Gain = [SA(select).Kp SA(select).Ki SA(select).Kd];
        SA(select).BestSol = sol;
        newsol.Gain = CreateNeighbor(sol.Gain,range_gains);
        SA(select).Kp  = newsol.Gain(1);
        SA(select).Ki =  newsol.Gain(2);
        SA(select).Kd =  newsol.Gain(3);
        
        SA(select).y = [];
        SA(select).subiter = 1;
        SA(select).iter = 1;
        
        SA(select).sol = sol;
        sys = [sol.Cost newsol.Gain(1) newsol.Gain(2) newsol.Gain(3) ];
    else
        if iter <= SA(select).iter_max
            newsol.Cost = CostFunction(SA(select).y,ref,Ts,SA(select).selectCostF);
            newsol.Gain = [SA(select).Kp SA(select).Ki SA(select).Kd];
            if newsol.Cost <= sol.Cost % verificamos si la nueva solicion es mejor
                sol = newsol;
            else % si no es mejor solucion
                DELTA = (newsol.Cost-sol.Cost)/sol.Cost;
                P = exp(-DELTA/Temp);
                if rand <= P
                    sol = newsol;
                end
            end
            %  actualizamos la mejor solucion encontrada
            if sol.Cost <= SA(select).BestSol.Cost
                SA(select).BestSol = sol;
            end
            SA(select).y = [];
            newsol.Gain = CreateNeighbor(sol.Gain,range_gains);  
            SA(select).Kp  = newsol.Gain(1);
            SA(select).Ki =  newsol.Gain(2);
            SA(select).Kd =  newsol.Gain(3);
            SA(select).sol = sol;
            sys = [SA(select).sol.Cost newsol.Gain(1) newsol.Gain(2) newsol.Gain(3) ];
            if iter == SA(select).iter_max
                    sys = [SA(select).BestSol.Cost SA(select).BestSol.Gain(1) SA(select).BestSol.Gain(2) SA(select).BestSol.Gain(3)];
            end
        end
    end
else
    SA(select).subiter = subiter;
    SA(select).iter = iter;
    SA(select).y = [SA(select).y y ];
    sys = [SA(select).sol.Cost SA(select).Kp SA(select).Ki SA(select).Kd ];
end
if iter > SA(select).iter_max
    sys = [SA(select).BestSol.Cost SA(select).BestSol.Gain(1) SA(select).BestSol.Gain(2) SA(select).BestSol.Gain(3)];
end


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];