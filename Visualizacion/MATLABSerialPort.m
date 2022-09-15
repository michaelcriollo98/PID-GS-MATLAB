function MATLABSerialPort(numero_muestras)
    close all;
    clc;
    warning ('off','all')
    if exist('numero_muestras','var') == 0
        numero_muestras = 2500;
    end
    port = 'COM3';
    %Inicializacion del puerto serial que se usara
    delete(instrfind({'Port'},{port}));
    puerto_serial = serial(port,'BaudRate',115200);
    %Apertura del puerto serial
    fopen(puerto_serial);
    disp('Apertura del puerto...');
    %contador de muestras
    k = 1;
try
    
    max_y1 = 25; min_y1 = 0;
    max_x1 = 200;
    
    max_y2 = 2; min_y2 = 0;
    max_x2 = 200;
    
    %nueva ventana de gráfica
    figure('Name', 'Serial communication: Matlab-STM32');
    
    subplot(2,1,1)
    title('SERIAL COMMUNICATION 1') ;
    xlabel('Número de muestra1');
    ylabel('Voltaje(V)1');
    grid on; hold on;
    ylim([ min_y1 max_y1]);
    
    text = fscanf (puerto_serial, '%f');
    if ischar(text)
        name_graf = split(text,'/');
        name_graf1 = split(name_graf{1});
        name_graf2 = split(name_graf{2});
        name_graf2 = name_graf2(1:end-1);
    else
       name_graf1 =  text(1:ceil(length( text)/2));
       name_graf2 = text(ceil(length( text)/2)+1:end);
    end
                    

    for i = 1:length(name_graf1)
      graf1(i).h =  animatedline('Color',dectobin(i),'LineWidth',0.6);
      graf1(i).y = [];
    end
    legend(['',name_graf1 ]);
    
    subplot(2,1,2) 
    title('SERIAL COMMUNICATION 2') ;
    xlabel('Número de muestra 2');
    ylabel('Voltaje(V)2');
    grid on; hold on;
    ylim([ min_y2 max_y2]);
   
   for i = 1:length(name_graf2)
       graf2(i).h =  animatedline('Color',dectobin(i),'LineWidth',0.6);
       graf2(i).u = [];
   end
   legend(['',name_graf2]);
   
   
        while k <= numero_muestras
            try
                aux_max1 = 2;  aux_min1 = -1;
                aux_max2 = 2;  aux_min2 = -1;
                
                valor = fscanf (puerto_serial, '%f');
                if ischar(valor)
                   error;
                end
                for i = 1:ceil(length(valor)/2)
                    graf1(i).y(k) = valor(i);
                    subplot(2,1,1)
                    addpoints(graf1(i).h,k,graf1(i).y(k)); 
                   
                    if k>max_x1
                        aux_max1= max( [aux_max1 graf1(i).y(k-max_x1:k)+2]);
                        aux_min1 = min([aux_min1 graf1(i).y(k-max_x1:k)-2]);                   
                    end

                end
                
                for i = 1:floor(length(valor)/2)
                    graf2(i).u(k) = valor(i+ceil(length(valor)/2));
                    
                    subplot(2,1,2)
                    addpoints(graf2(i).h,k,graf2(i).u(k)); 
                   
                    if k>max_x1
                        aux_max2 = max([aux_max2 graf2(i).u(k-max_x2:k)+2]);
                        aux_min2 = min([aux_min2 graf2(i).u(k-max_x2:k)-2]);                   
                    end
                end

%--------------------------------------mover ejes--------------------------
                
                if k <= max_x1 
                    subplot(2,1,1)
                    xlim([0 max_x1 ]); 
                else
                    subplot(2,1,1)
                    xlim([k-max_x1  k]);  
                end
                
                if k <= max_x2
                    subplot(2,1,2)
                    xlim([0 max_x2 ]); 
                else
                    subplot(2,1,2)
                    xlim([k-max_x2  k]);  
                end
                

                if k<= max_x1
                    aux1 = max(valor(1:ceil(length(valor)/2)));
                    if aux1>max_y1 
                       max_y1 = aux1 +5; 
                    end
                    aux1 = min(valor(1:ceil(length(valor)/2)));
                    if aux1<min_y1
                       min_y1 =  aux1-5;
                    end
                else
                    min_y1 = aux_min1;
                    max_y1 = aux_max1;
                end
                
                
                if k<= max_x2
                    aux2 = max(valor(ceil(length(valor)/2)+1:length(valor)));
                    if aux2>max_y2 
                       max_y2 = aux2+5; 
                    end
                    aux2 = min(valor(ceil(length(valor)/2)+1:length(valor)));
                    if aux2<min_y2
                       min_y2 =  aux2-5;
                    end
                else
                    min_y2 = aux_min2;
                    max_y2 = aux_max2;
                end

%-------------------------------------------------------------------------
                subplot(2,1,1)
                ylim([min_y1 max_y1]);
                
                subplot(2,1,2)
                ylim([min_y2 max_y2]);
                
                drawnow
              
                k=k+1;
            catch
                if ischar(valor)
                   name_graf = split(valor,'/');
                   name_graf1 = split(name_graf{1});
                   name_graf2 = split(name_graf{2},' ');
                   name_graf2 = name_graf2(1:end-1);                   
                   subplot(2,1,1) 
                   legend(['',name_graf1 ]);
                   subplot(2,1,2) 
                   legend(['',name_graf2 ]);
                else
                    break;
                end
            end
        end
    %Cierro la conexión con el puerto serial y elimino las variables
catch
       disp('Existe un error');
end
    warning ('on','all')
    fclose(puerto_serial);
    delete(puerto_serial);
    figure (2)
    save('Altura_AS-PID_FallaT2_p3_1.mat','graf1')
    save('Ucontrol_AS-PID_FallaT2_p3_1','graf2')
    clear ;
    close all;
    disp('Puerto cerrado');
    disp('Finalizo...');
end

%**** la p3 encuntra lso valores p4 solo los valores
function b = dectobin(num)
   a = dec2bin(num,3);
   b = [];
   for j = 1:3
    b = cat(1,b, str2double(a(j)));    
   end
   b = b';
end