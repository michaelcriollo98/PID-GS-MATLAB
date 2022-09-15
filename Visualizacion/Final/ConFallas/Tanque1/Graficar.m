clear ;
close all;
clc;
filename1 = "Altura_AS-PID_FallaT1_p4";
filename2 = "Ucontrol_PID_FallaT1_p1";
signal1 = load(filename1);
signal2 = load(filename2);

H1 = signal1.graf1(1).y;
H2 = signal1.graf1(2).y;

%H1 =  medfilt1(H1,5);
%H2 =  medfilt1(H2,5);
Ref1 = signal1.graf1(3).y;
Ref2 = signal1.graf1(4).y;

u1 = signal2.graf2(1).u;

enable1 =  Ref1(2:end)- Ref1(1:end-1); 
enable2 = abs( Ref2(2:end)- Ref2(1:end-1));
t = 0:1:length(H2)-1;


subplot(2,1,1)
plot(t,H1(1:length(t)),t,Ref1,'LineWidth',0.6);
title('Salida del Sistema') ;
xlabel('Tiempo (seg)');
ylabel('Distancia (cm)');
grid on; hold on;
legend({'H1','Ref1'});
subplot(2,1,2) 
plot(t,H2,t,Ref2,'LineWidth',0.6);
title('Salida del Sistema') ;
xlabel('Tiempo (seg)');
ylabel('Distancia (cm)');
grid on; hold on;
legend({'H2','Ref2'});

figure (2)
subplot(2,1,1)
plot(enable1)
subplot(2,1,2) 
plot(enable2)

muestras = 100;
y1 = [];
y2 = [];
j = 1;
k = 1;
entro = 1;
txt1 = {};
dim = [.2 .5 .3 .3];
sumJ1 = [];
sumOS1 = [];
sumts1 = [];
sumIAE1 = [];
sumJ2 = [];
sumOS2 = [];
sumts2 = [];
sumIAE2 = [];
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 5;

for i=1:min(length(H1),length(H2))-2
    if entro == 0
        if (j< 140)
                y1(k,j) =H1(i); 
                y2(k,j) =H2(i); 
                j=j+1;
        else
            entro = 1;
        end
    end
    if (enable1(i) > 0  )
        if (i>350)
            %y1(k,:) =  medfilt1(y1(k,:),10);
           [J,OS,ts,IAE] = CostFunction(y1(k,:),5,2,1);
           if J ~= inf
               sumJ1 = [sumJ1 J]
           end
           if ~isnan(ts)
               sumts1 = [sumts1 ts]
           end
               sumOS1 = [sumOS1 OS]
               sumIAE1 =[sumIAE1 IAE]
           txt1{k} = {['J: ' num2str(J)], ['OS:' num2str(OS)],[ 'ts:' num2str(ts)],[ 'IAE:' num2str(IAE)]};
          % y1(k,:) =  medfilt1(y2(k,:),10);
           [J,OS,ts,IAE] = CostFunction(y2(k,:),5,2,1);
           if J ~= inf
               sumJ2 = [sumJ2 J]
           end
           if ~isnan(ts)
               sumts2 = [sumts2 ts]
           end
           sumOS2 = [sumOS2 OS]
           sumIAE2 =[sumIAE2 IAE]
           txt2{k} = {['J: ' num2str(J)], ['OS:' num2str(OS)],[ 'ts:' num2str(ts)],[ 'IAE:' num2str(IAE)]};
           k  = k+1;
        end
        j = 1;
        entro = 0;
    end
end
for m = 1 : k-1
    figure (m+2)
    plot(y1(m,:))
    annotation('textbox',dim, ...
    'String',txt1{m},'EdgeColor','none')
    w = m+2;
end

for m = 1 : k-1
    figure (w+m)
    plot(y2(m,:))
    annotation('textbox',dim, ...
    'String',txt2{m},'EdgeColor','none')
end

J1 = sum(sumJ1)/length(sumJ1)
OS1 = sum(sumOS1)/length(sumOS1)
ts1 = sum(sumts1)/length(sumts1)
IAE1 = sum(sumIAE1)/length(sumIAE1)


J2 = sum(sumJ2)/length(sumJ2)
OS2 = sum(sumOS2)/length(sumOS2)
ts2 = sum(sumts2)/length(sumts2)
IAE2 = sum(sumIAE2)/length(sumIAE2)

function [J,OS,ts,IAE] = CostFunction(y,ref,select_costF,dt)
muestras = length(y);
yfinal = 3;
yinit = 3;
t = linspace(0,(muestras-1)*dt,muestras);
e = ref-y;

S1 = stepinfo(y-y(1),t,yfinal,'SettlingTimeThreshold',0.05)
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
        J = 0.05*IAE + 0.05*ts + 0.90*OS;
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
end

    