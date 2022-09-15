function [sys,x0,str,ts,simStateCompliance] = sfun_Astar(t,x,u,flag,AS_vars)
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
    sys=mdlOutputs(t,x,u,AS_vars);
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
sizes.NumInputs      = 4;
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

function sys=mdlOutputs(t,x,u,AS_vars)
global AS
select = AS_vars(1);
    Ts = AS_vars(2);
    
       e = u(1);
     ref = u(2);
  enable = u(3);
    iter = u(4);

y = ref - e;

if AS(select).fin == 0
    if enable == 1 && length(AS(select).y) > 1
        dim = size(AS(select).space);
        y_cifras = 10^length(num2str(dim(1)));
        z_cifras = 10^(length(num2str(dim(1))) + length(num2str(dim(2))));
        salir = 0;
        while salir == 0
            posActual = [AS(select).X1 AS(select).Y1 AS(select).Z1];
            if ~( (AS(select).X1 == AS(select).X2 && AS(select).Y1 == AS(select).Y2 ...
                 && AS(select).Z1 == AS(select).Z2)|| AS(select).iter > AS(select).iter_max)
                 if (AS(select).cont == 0)
                     cont_next = 1;
                     AS(select).pause = 1;
                     coord = posActual;
                 else
                    AS(select).cont   = delimitar_coord(dim,posActual,AS(select).cont );
                    cont_next = delimitar_coord(dim,posActual,AS(select).cont +1);
                    subiter = contador(AS(select).cont);
                    coord = posActual + subiter -2 ;
                 end
                 x = coord(1);
                 y = coord(2);
                 z =coord(3);
%-----------------------------------------------------------------------------------------------------------------------
                        if AS(select).space(x,y,z) == 0 && ...
                            isempty(find(AS(select).ListaC == AS(select).ID(x,y,z), 1)) == 1
                        
                            pos_a = find(AS(select).ListaA == AS(select).ID(x,y,z),1); 
                            if isempty(pos_a) == 1 % si tiene nuevos vecino

                                if AS(select).pause == 1
                                    % calculamos el costo G y la heuristica H
                                    AS(select).H(x,y,z)  = CostFunction(AS(select).y,ref,Ts,AS(select).selectH );
                                    AS(select).G(x,y,z) = AS(select).G(AS(select).X1,AS(select).Y1,AS(select).Z1)+ ...
                                        sqrt((x-AS(select).X1)^2+(y-AS(select).Y1)^2+(z-AS(select).Z1)^2); 
                                    AS(select).F(x,y,z) = AS(select).G(x,y,z) + AS(select).H(x,y,z);
                                    % idicamos de donde provino 
                                    AS(select).padre(x,y,z) = AS(select).ID(AS(select).X1,AS(select).Y1,AS(select).Z1);
                                    % agreamos a la lista de pixel de vecinos conocidos
                                    AS(select).ListaA = cat(1,AS(select).ListaA,AS(select).ID(x,y,z)) ;

                                    AS(select).ListaAF =cat(1,AS(select).ListaAF, AS(select).F(x,y,z));
                                    AS(select).pause = 0;
                                    AS(select).y = [];
                                else
                                    AS(select).pause = 1;
                                    salir = 1;
                                end
                            else 
                              G = AS(select).G(AS(select).X1,AS(select).Y1,AS(select).Z1)+...
                                  sqrt((x-AS(select).X1)^2+(y-AS(select).Y1)^2)+(z-AS(select).Z1)^2;
                            
                              if G < AS(select).G(x,y,z)
                                 AS(select).G(x,y,z) = G;
                                 AS(select).padre(x,y,z) = AS(select).ID(AS(select).X1,AS(select).Y1,AS(select).Z1);    % si el gasto es menor 
                                 AS(select).F(x,y,z) = AS(select).G(x,y,z) + AS(select).H(x,y,z); % colocamos su nuevo padre
                                 AS(select).ListaAF(pos_a) = AS(select).F(x,y,z);     % y costo 
                              end
                            end
                        end
                        if (((mod(AS(select).cont-1,27)>mod(cont_next-1,27)) && AS(select).pause == 0 )||AS(select).cont == 0)
                    
                            ID_XYZ = find(AS(select).ListaAF == min(AS(select).ListaAF),1);    % buscamos su indentificador
                     
                            if  isempty(ID_XYZ) ~= 1                       % si se encontro la posicion del valor minimo 
                                ZYX = AS(select).ListaA(ID_XYZ);          % determinamos las cordenadas
                                AS(select).Z1 = fix(ZYX / z_cifras);      
                                YX = ZYX - AS(select).Z1 * z_cifras;
                                AS(select).Y1 = fix( YX / y_cifras);
                                AS(select).X1 =  YX - AS(select).Y1 * y_cifras;
                                % quitamos el elemento encontrado de la lista de vecinos
                                % abiertos y colocamos en los cerrados
                                AS(select).ListaC = cat(1,AS(select).ListaC,AS(select).ID(AS(select).X1,AS(select).Y1,AS(select).Z1));   
                                AS(select).ListaA  = [AS(select).ListaA(1:ID_XYZ-1,1); AS(select).ListaA(ID_XYZ+1:end,1)];
                                AS(select).ListaAF  = [AS(select).ListaAF(1:ID_XYZ-1,1); AS(select).ListaAF(ID_XYZ+1:end,1)];
                                if (length(AS(select).ListaC) == dim(1)*dim(2)*dim(3))
                                    AS(select).fin =1;
                                    salir = 1;
                                end
                            end
                        end
                        if (AS(select).pause == 0)
                           AS(select).cont = AS(select).cont+1; 
                        end

            else
                AS(select).fin =1;
                salir = 1;
            end

        end
        if AS(select).fin == 1
            [x_min, yz_min] = find( AS(select).H == min(AS(select).H,[],'all'),1);
            y_min = (mod(yz_min-1,dim(2))+1);
            z_min = fix((yz_min-1)/ dim(2))+1;
            AS(select).Kp = AS(select).Kp_values(x_min) ;
            AS(select).Kd = AS(select).Kd_values(y_min);
            AS(select).Ki = AS(select).Ki_values(z_min);
        else
            AS(select).Kp = AS(select).Kp_values(x) ;
            AS(select).Kd = AS(select).Kd_values(y);
            AS(select).Ki = AS(select).Ki_values(z);
        end
        sys = [ AS(select).Kp AS(select).Ki AS(select).Kd ];
    else
        AS(select).iter = iter;
        AS(select).y = [AS(select).y y ];
        sys = [ AS(select).Kp AS(select).Ki AS(select).Kd ];
    end
else
    sys = [ AS(select).Kp AS(select).Ki AS(select).Kd ];
end     

    
  function iter = delimitar_coord(dim,posActual,iter)
    while true
      subiter = contador(iter);
      coord = posActual + subiter -2;
       for i = 1:3
          if (coord(i)<=0 ||coord(i)>dim(i))
              iter = iter + 3^(i-1);
          end
       end
      if ~(coord(1)<=0 ||coord(1)>dim(1) ||coord(2)<=0 ||coord(2)>dim(2) ||coord(3)<=0 ||coord(3)>dim(3)  )
          break;
      end
    end



function subiter = contador(iter)
    base = 3;cifras = 3;
    cont = zeros(1,cifras);
    subiter = zeros(1,cifras);
    for i = 1:cifras
        cont(i) = mod((iter -1),base^i)+1;
        subiter(i) = fix((cont(i)-1)/(base^(i-1)))+1;
    end

    
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];