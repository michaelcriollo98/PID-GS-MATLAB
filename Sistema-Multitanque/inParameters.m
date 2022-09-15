clc
clear -except SA AS;
close all;
global  SA AS

% Parametros
% Área de la sección transversal de los tanques 1 y 2.
L1 = 0.20;              A1 = L1^2;
L2 = 0.20;              A2 = L2^2;

% Porcentaje de apertura de las válvulas 1, 2 y 3 ( 0 < u1,2,3 < 1 ).
ue1 = 0.30;
ue2 = 0.50;
ue3 = 0.35;

% area de la sección transversal de los tubos 
% conectados a las válvulas   [m^2]            
a2 = 4.380e-5;  
a3 = 4.601e-5;

% Aceleración de la gravedad  [kg/(m*s^2)]
g = 9.81; 
% Caudal nominal de la bomba  [m^3/s]
q0 = 4*(0.001/60);  

% Nivel de agua de los tanques 1 y 2.
he1 = (1/(2*g))*(q0*ue1/(a2*ue2))^2;
he2 = he1*(a2*ue2/(a3*ue3))^2;
he1 = 0;
he2 = 0;
%area de la fuga
rf = 3.175*10^-3;             af = pi*(rf)^2;
qout = 1;

% Potencia del ruido AWG
N =  0e-7;

%parametros PID

Kp1 = 0.2;
Kd1 = 0.01; 
Ki1 = 0.000575 ;    %Ki1 = 0.0005 ; 

Gains1 = [Kp1 Ki1 Kd1];

Kp2 = 0.2;
Kd2 = 0.1; 
Ki2 = 0.0007 ;    %  Ki = 0.0001;

Gains2 = [Kp2 Ki2 Kd2];

Kp3 = 0.7;
Kd3 = 0.1;  
Ki3 = 0.0015;    %  Ki = 0.001;

Gains3 = [Kp3 Ki3 Kd3];

%Tiempo de muestreo
Ts = 1;
% varizbles del modelo de los tanques
vars = [a2,a3, A1, A2,q0,g,af,Ts,he1,he2];

% *****************  Fuzzy ***********************
%  ******************* FGS1 **********************

Kp1min = 0.533 * Kp1; 
Kp1max =    1  * Kp1; 
Kd1min = 1.067 * Kd1; 
Kd1max =     2 * Kd1; 

Ki1min =  0.01 * Ki1;
Ki1max =  0.85 * Ki1;


range_gains1 = [Kp1min Kp1max Kd1min Kd1max Ki1min Ki1max];

Kp2min = 0.533 * Kp2; 
Kp2max =    1  * Kp2; 
Kd2min = 1.067 * Kd2; 
Kd2max =     2 * Kd2; 

Ki2min =  0.01 * Ki2;
Ki2max =  0.85 * Ki2;

range_gains2 = [Kp2min Kp2max Kd2min Kd2max Ki2min Ki2max];


Kp3min = 0.533 * Kp3; 
Kp3max =    1  * Kp3; 
Kd3min = 1.067 * Kd3; 
Kd3max =     2 * Kd3; 

Ki3min =  0.01 * Ki3;
Ki3max =  0.85 * Ki3;

range_gains3 = [Kp3min Kp3max Kd3min Kd3max Ki3min Ki3max];


global fis
try
    fis = readfis('fuzzy_control');
catch
end
% ******************** SA *************************************************
% *************************************************************************
T0 = 0.025;       % Temperatura inicial
alpha = 0.99;     % Temp. Reduction Rate
   
reset  = exist('reset.mat','file');

% numero de iteraciones que se dan para reinciar la busqueda
reinciar =  inf;     %(default inf) 
    
if reset ~=0 
    try
    if SA(1).iter >= reinciar
      delete reset.mat
      reset = 0;
    end
    catch
      reset = 0;  
    end
end

if  reset == 0
    sol.Cost  = 0;
    sol.Gain  = [1 1 1];
    
    SA(1).subiter_max = 5;
    SA(1).iter_max = 1;
    SA(1).subiter = 0;
    SA(1).iter = 0;
    SA(1).Kp = Kp1;
    SA(1).Kd = Kd1;
    SA(1).Ki = Ki1;
    SA(1).BestSol = sol;
    SA(1).sol = sol;
    SA(1).y = [];
    SA(1).selectCostF = 2;

    SA(2) = SA(1);
    SA(3) = SA(2);

    SA(2).Kp = Kp2; SA(3).Kp = Kp3;
    SA(2).Kd = Kd2; SA(3).Kd = Kd3;
    SA(2).Ki = Ki2; SA(3).Ki = Ki3;
    
    SA(2).iter_max = 6;
    SA(3).iter_max = 6;
end


SA_vars1 = [alpha T0 1 Ts range_gains1];
SA_vars2 = [alpha T0 2 Ts range_gains2];
SA_vars3 = [alpha T0 3 Ts range_gains3];

%**************************************************************************
%********************************A-star************************************
 
reset  = exist('deletetoreset.mat','file');

if reset ~=0 

    try
        if AS(1).iter >= reinciar
          delete deletetoreset.mat
          reset = 0;
        end
    catch
      reset = 0;  
    end

end
if  reset == 0
     
    dim11 = 4; dim12 = 4; dim13 = 4;
    dim21 = 4; dim22 = 4; dim23 = 4;
    dim31 = 4; dim32 = 4; dim33 = 4;
    
    [AS(1).Kp_values,AS(1).Kd_values,AS(1).Ki_values, ...
    AS(1).space, AS(1).ID, AS(1).X1, AS(1).Y1, AS(1).Z1,...
    AS(1).X2, AS(1).Y2, AS(1).Z2] = Astar_Init(dim11,dim12,dim13,Gains1,range_gains1);
    
    AS(1).iter_max = 10;
    AS(1).cont = 0;
    AS(1).fin = 0;
    AS(1).pause = 0;
    AS(1).iter = 0;
    AS(1).Kp = Kp1;
    AS(1).Kd = Kd1;
    AS(1).Ki = Ki1;
    AS(1).y = [];
    AS(1).selectH = 2;

    [AS(1).ListaA,AS(1).ListaAF, ...
     AS(1).ListaC ,AS(1).padre,...
     AS(1).F,AS(1).G,AS(1).H] = Astar_param(dim11,dim12,dim13);

    AS(2) = AS(1);
    AS(3) = AS(2);

    AS(2).Kp = Kp2; AS(3).Kp = Kp3;
    AS(2).Kd = Kd2; AS(3).Kd = Kd3;
    AS(2).Ki = Ki2; AS(3).Ki = Ki3;
    
    [AS(2).Kp_values,AS(2).Kd_values,AS(2).Ki_values, ...
    AS(2).space, AS(2).ID, AS(2).X1, AS(2).Y1, AS(2).Z1,...
    AS(2).X2, AS(2).Y2, AS(2).Z2] = Astar_Init(dim21,dim22,dim23,Gains2,range_gains2);

    [AS(2).ListaA,AS(2).ListaAF, ...
     AS(2).ListaC ,AS(2).padre,...
     AS(2).F,AS(1).G,AS(2).H] = Astar_param(dim21,dim22,dim23);

    [AS(3).Kp_values,AS(3).Kd_values,AS(3).Ki_values, ...
    AS(3).space, AS(3).ID, AS(3).X1, AS(3).Y1, AS(3).Z1,...
    AS(3).X2, AS(3).Y2, AS(3).Z2] = Astar_Init(dim31,dim32,dim33,Gains3,range_gains3);

    [AS(3).ListaA,AS(3).ListaAF, ...
     AS(3).ListaC ,AS(3).padre,...
     AS(3).F,AS(1).G,AS(3).H] = Astar_param(dim31,dim32,dim33);

    AS(2).iter_max = 40;
    AS(3).iter_max = 40;
end

AS_vars1 = [1 Ts];
AS_vars2 = [2 Ts];
AS_vars3 = [3 Ts];

%--------------------------------------------------------------------------
function  ID = GenerateID(dim1,dim2,dim3)
    ID = zeros(dim1,dim2,dim3);% matriz de ceros  
    x_cifras = 1;
    y_cifras = 10^length(num2str(dim1));
    z_cifras = 10^(length(num2str(dim1)) + length(num2str(dim2)));

        for i = 1:dim1        % para cada fila de la matriz
            for j = 1:dim2    % para cada columna de la matriz
                for k = 1:dim3
                    % el numero de identifación es una transfomacion linal 
                    % de a fila i columna j 
                       ID(i,j,k) = i*x_cifras + j * y_cifras + k * z_cifras;  
                end
            end 
        end
end

function [Kp_values,Kd_values,Ki_values, space, ID, X1, Y1, Z1, X2, Y2, Z2] = Astar_Init(dim1,dim2,dim3,gains,range_gains)

Kp = gains(1);
Ki = gains(2);
Kd = gains(3);

Kpmin = range_gains(1);
Kpmax = range_gains(2);
Kdmin = range_gains(3);
Kdmax = range_gains(4);
Kimin = range_gains(5);
Kimax = range_gains(6);

Kp_values = linspace(Kpmin,Kpmax,dim1);
Kd_values = linspace(Kdmin,Kdmax,dim2);
Ki_values = linspace(Kimin,Kimax,dim3);
space = zeros(length(Kp_values) ,length(Kd_values),length(Ki_values));

ID = GenerateID(dim1, dim2, dim3);

X1 = find( Kp_values == Kp);
Y1 = find( abs(Kd_values-Kd)<0.1,1);
Z1 = find( abs(Ki_values-Ki)<0.3,1);
if (isempty(X1)||isempty(Y1)||isempty(Z1))
    X1=1;Y1=1;Z1=1;
end

X2 = randi(dim1);
Y2 = randi(dim2);
Z2 = randi(dim3);
X2 = -1;
Y2 = -1;
Z2 = -1;

end

function [ListaA,ListaAF, ListaC ,padre,F,G,H] = Astar_param(dim1,dim2,dim3)
ListaA = [];
ListaAF = [];
ListaC = [];
padre = zeros(dim1,dim2,dim3);
F = nan*ones(dim1,dim2,dim3);         % F = G + H
G = zeros(dim1,dim2,dim3);            % Costo
H = nan*ones(dim1,dim2,dim3);         % Heurística
end



