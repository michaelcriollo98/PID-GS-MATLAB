clearvars -except GSPID
close all
% load('reset.mat')
%  SAGains = [BestGains1 ;BestGains2'; BestGains3];
% load('deletetoreset.mat')
% ASGains = [BestGains1 ;BestGains2; BestGains3];
% save('FallasT2ResultadosPID.mat','GSPID','SAGains','ASGains');
graf1 = GSPID.signals(1).values;
graf2 = GSPID.signals(2).values;

%graf1 = graf1/25*100;
%graf2 = graf2/25*100;


ref1 = graf1(:,1);
h1_PID = graf1(:,2);
h1_FGSPID = graf1(:,3);
h1_SAPID = graf1(:,4);
h1_ASPID = graf1(:,5);


ref2 = graf2(:,1);
h2_PID = graf2(:,2);
h2_FGSPID = graf2(:,3);
h2_SAPID = graf2(:,4);
h2_ASPID = graf2(:,5);

init = 1980;
fin = 4500;

H1 = h1_FGSPID(init:fin);
H2 = h2_FGSPID(init:fin);
Ref1 = ref1(init:fin);
Ref2 = ref2(init:fin);

enable2 =  ref1(init+1:fin)- ref1(init:fin-1); 
enable1 =  ref2(init+1:fin)- ref2(init:fin-1);

figure (2)
subplot(2,1,1)
plot(enable1)
subplot(2,1,2) 
plot(enable2)

figure(3)

muestras = 499;
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

graf1 = graf1/25*100;
graf2 = graf2/25*100;


ref1 = graf1(:,1);
h1_PID = graf1(:,2);
h1_FGSPID = graf1(:,3);
h1_SAPID = graf1(:,4);
h1_ASPID = graf1(:,5);


ref2 = graf2(:,1);
h2_PID = graf2(:,2);
h2_FGSPID = graf2(:,3);
h2_SAPID = graf2(:,4);
h2_ASPID = graf2(:,5);


SettlingMin = 7.7 /25*100;
SettlingMax = 8.3 /25*100 ;


H1 = h1_ASPID;
H2 = h2_ASPID;
name = 'A*-PID';

t = 0:1:length(ref1)-1;
figure (1)
subplot(2,1,1)
plot(t(init:fin)-init,ref1(init:fin),'r--',t(init:fin)-init,H1(init:fin),'b','LineWidth',1.2);
hold on
plot(t(init:fin)-init,SettlingMin*ones(1,fin-init+1),'k:',t(init:fin)-init,SettlingMax*ones(1,fin-init+1),'k:','LineWidth',0.1);
axis([-inf inf 0 50 ])
title('Respuesta del sistema - Tanque 1','Interpreter','latex') ;
xlabel('Tiempo [seg]','Interpreter','latex');
ylabel('Nivel [\%]','Interpreter','latex');
legend({'Referencia',name,},'Interpreter','latex');
ax = gca;
ax.TickLabelInterpreter = 'latex';
subplot(2,1,2) 
plot(t(init:fin)-init,ref2(init:fin),'r--',t(init:fin)-init,H2(init:fin),'b','LineWidth',1);
hold on
plot(t(init:fin)-init,SettlingMin*ones(1,fin-init+1),'k:',t(init:fin)-init,SettlingMax*ones(1,fin-init+1),'k:','LineWidth',0.1);
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