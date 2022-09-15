clear ;
close all;
clc;
filename1 = "Altura_AS-PID_p1";
filename2 = "Ucontrol_PID_1_p2";
name = 'A*-PID';

signal1 = load(filename1);
signal2 = load(filename2);

H1 = signal1.graf1(1).y;
H2 = signal1.graf1(2).y;

H1 = medfilt1(H1,5);
H2 = medfilt1(H2,5);

ref1 = signal1.graf1(3).y;
ref2 = signal1.graf1(4).y;

u1 = signal2.graf2(1).u;

init = 1;
fin = length(ref1);

Ref1 = ref1(init:fin);
Ref2 = ref2(init:fin);

enable1 =  ref1(init+1:fin)- ref1(init:fin-1); 
enable2 =  ref2(init+1:fin)- ref2(init:fin-1);

figure (2)
subplot(2,1,1)
plot(enable1)
subplot(2,1,2) 
plot(enable2)

figure(3)

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
inicio = 1;

for i=1:min(length(H1),length(H2))-2
    if entro == 0
        if (j< muestras )
                y1(k,j) =H1(i); 
                y2(k,j) =H2(i); 
                j=j+1;
        else
            entro = 1;
        end
        inicio = 0;
    end
    if (abs(enable1(i)) > 0  || i ==min(length(H1),length(H2))-2)
        if (inicio == 0)
           [J,OS,ts,IAE] = CostFunction(y1(k,:),Ref1(i-1),2,1);
           if J ~= inf
               sumJ1 = [sumJ1 J]
           end
           if ~isnan(ts)
               sumts1 = [sumts1 ts]
           end
               sumOS1 = [sumOS1 OS]
               sumIAE1 =[sumIAE1 IAE]
           txt1{k} = {['J: ' num2str(J)], ['OS:' num2str(OS)],[ 'ts:' num2str(ts)],[ 'IAE:' num2str(IAE)]};
           [J,OS,ts,IAE] = CostFunction(y2(k,:),Ref2(i-1),2,1);
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



%***********************************************

signal1.graf1(1).y;

H1 = H1/25*100;
H2 = H2/25*100 ;
ref1 = ref1/25*100 ;
ref2 = ref2/25*100 ;
t = 0:1:length(ref1)-1;
SettlingMin = 7.7 /25*100;
SettlingMax = 8.3 /25*100 ;


init1 = 636;
fin1 = 925;

init2 = 926;
fin2 = 1215;

init3 = 1216;
fin3 = 1362;


init4 = 252;
fin4 = 451;

init5 = 1252;
fin5 = 1451;

init6 = 1452;
fin6 = 1551;

% 
% 
ref1F = [ref1(init1:fin1) ref1(init2:fin2) ref1(init3:fin3)];
ref2F = [ref2(init4:fin4) ref2(init5:fin5) ref2(init6:fin6)];
H1F = [H1(init1:fin1) H1(init2:fin2) H1(init3:fin3)];
H2F = [H2(init4:fin4) H2(init5:fin5) H2(init6:fin6)];

t1 = 0:1:length(H1F)-1;
t2 = 0:1:length(H2F)-1;

figure (1)
subplot(2,1,1)
%plot(t(init:fin)-init,ref1(init:fin),'r--',t(init:fin)-init,H1(init:fin),'b','LineWidth',1.2);
plot(t1,ref1F,'r--',t1,H1F,'b','LineWidth',1.2);
hold on
%plot(t(init:fin)-init,SettlingMin*ones(1,fin-init+1),'k:',t(init:fin)-init,SettlingMax*ones(1,fin-init+1),'k:','LineWidth',0.1);
plot(t1,SettlingMin*ones(1,length(H1F)),'k:',t1,SettlingMax*ones(1,length(H1F)),'k:','LineWidth',0.1);

axis([-inf inf 0 50 ])
title('Respuesta del sistema - Tanque 1','Interpreter','latex') ;
xlabel('Tiempo [seg]','Interpreter','latex');
ylabel('Nivel [\%]','Interpreter','latex');
legend({'Referencia',name,},'Interpreter','latex');
ax = gca;
ax.TickLabelInterpreter = 'latex';
subplot(2,1,2) 
%plot(t(init:fin)-init,ref2(init:fin),'r--',t(init:fin)-init,H2(init:fin),'b','LineWidth',1);
plot(t2,ref2F,'r--',t2,H2F,'b','LineWidth',1.2);
hold on
%plot(t(init:fin)-init,SettlingMin*ones(1,fin-init+1),'k:',t(init:fin)-init,SettlingMax*ones(1,fin-init+1),'k:','LineWidth',0.1);
plot(t2,SettlingMin*ones(1,length(H2F)),'k:',t2,SettlingMax*ones(1,length(H2F)),'k:','LineWidth',0.1);
axis([-inf inf 0 50 ])
ax = gca;
ax.TickLabelInterpreter = 'latex';
title('Respuesta del sistema - Tanque 2','Interpreter','latex') ;
xlabel('Tiempo [seg]','Interpreter','latex');
ylabel('Nivel [\%]','Interpreter','latex');
hold on;
legend({'Referencia',name,},'Interpreter','latex');


function [J,OS,ts,IAE] = CostFunction(y,ref,select_costF,dt)
muestras = length(y);
yfinal = ref
yinit = y(1);
ynorm = (y - yinit)/(yfinal-yinit)
plot(ynorm);

t = linspace(0,(muestras-1)*dt,muestras);
e = ref-y;

S1 = stepinfo(ynorm,t,1,'SettlingTimeThreshold',0.05);
ts = S1.SettlingTime;
OS = S1.Overshoot;
tr = S1.RiseTime;

ISE = trapz(t,e.^2);
IAE = trapz(t,abs(e));
ITAE = trapz(t,t.*(abs(e)));
ITSE  = trapz(t',(t.*(e.^2))');
    
switch select_costF
    case 1
        J = 0.10*ITAE + 0.60*OS + 0.20*ts + 0.10*tr;
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