function [sys,x0]=sfun_tankmodel(t,x,u,flag,vars)
% En el Flag 0 es el paso donde indicamos para la S-Function que es lo que
% la función va a encontrar en el momendo de leer el modelo. Esa
% información será encontrada en un vector de 6 elementos, que llamaremos
% "sys"
if flag==0
   %Elemento 1: Número de estados continuos (Ecuaciones diferenciables)= 2
   %Elemento 2: Número de estados discretos: 0
   %Elemento 3: Número de saídas del modelo: 2
   %Elemento 4: Número de Entradas del modelo (u1)...(u3): 3 entradas
   %Elemento 5: Parametro de control, colcar 0
   %Elemento 6: Tipologia do processo, (1 para processos continuos)
   [sys]=[2,0,2,7,0,0];
   %condiciones iniciales
   he1 = vars(9);
   he2 = vars(10);
   x0=[he1 he2];
elseif flag==1
    %Flag 1 llama el modelo
    g = vars(6);
    af = vars(7);
    Ts = vars(8);  
    
    U = u(1:3);
    X = u(4:5);
    kl = u(6:7);  
    
    l1 = kl(1) * af * sqrt(2*g*X(1));
    l2 = kl(2) * af * sqrt(2*g*X(2)); 
    
   [sys]=tanksmodel(t,X,U,l1,l2,vars);
   
elseif flag==3
    %Flag 3 indica la respuesta que se debe obtener, en este caso, un vetor con las 2
    %variables de estado
   [sys]=x;
else
    %Como paso final con un vector nulo se cierra el bucle
   [sys]=[];
end
end