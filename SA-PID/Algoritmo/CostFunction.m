function J = CostFunction(y,ref,select_costF,dt)
muestras = length(y);
yfinal = ref;

t = linspace(0,(muestras-1)*dt,muestras);
e = ref-y;

S1 = stepinfo(y,t,yfinal,'SettlingTimeThreshold',0.05);
ts = S1.SettlingTime;
OS = S1.Overshoot;
tr = S1.RiseTime;

ISE = trapz(t,e.^2);
IAE = trapz(t,abs(e));
ITAE = trapz(t,t.*(abs(e)));
ITSE  = trapz(t',(t.*(e.^2))');
    
switch select_costF
    case 1
        J = 0.10*IAE + 0.60*OS + 0.20*ts + 0.10*tr;
    case 2
        J = 0.05*ITAE + 0.05*ts + 0.90*OS;
    case 3
        J = 0.10*ISE + 0.20*ts + 0.70*tr;
    case 4
        J = 0.10*ISE + 0.70*ts + 0.20*tr;
    case 5
        J = 0.10*ISE + 0.10*ts + 0.80*OS;
    otherwise
        J = 0.10*ITSE + 0.10*ts + 0.80*OS;
end

if (isnan(J) || J == 0)
    J = inf;
end
