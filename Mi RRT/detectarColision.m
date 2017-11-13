

function [qnew,num_llamadas] = detectarColision(qnear,qrand,qmax,paso)

%% DECLARACION DE VARIABLES
    % Variable que recoge las llamadas al colisionador
    num_llamadas = 0;   
    
    % Direccion desde el nodo del arbol hasta el nuevo nodo aleatorio
    direccion = (qrand-qnear)/norm(qrand-qnear);    
    
    % Distancia maxima entre el nodo cercano del arbol y el nodo aleatorio
    dist_max = norm(qrand-qnear);   
    if dist_max<0
        paso=-paso; % Si la distancia es negativa, el paso también lo será
    end
    
    n=0;    % Contador de nuevos nodos
    
%% BUCLE DE COLISION
for i=0:paso:(dist_max+paso)
    
    % Se ajusta el iterador a la distancia maxima cuando el iterador sea
    % mayor que la distancia maxima
    if i>dist_max
        i=dist_max;
    end
    
    % Se añade un nuevo nodo al contador
    n=n+1;
    
    % Se obtiene el siguiente nodo a analizar
    qaux.nodo(n,:) = qnear + i*direccion;

    % Se comprueba que el nodo está dentro de la zona de trabajo del robot
    for j = 1:size(qaux.nodo,2)
        % Si sobrepasa los límites devuelve la lista de nodos hasta el
        % anterior al actual (que está fuera de los limites)
        if abs(qaux.nodo(n,j)) > qmax(j)
            qnew=qaux.nodo(1:n-1,:);
            % Se cierra el bucle para devolver qnew
            break
        end    
    end
    
    % Se comprueba si hay colision del nodo con el entorno
    colision = apoloCheckRobotJoints('ASEA', qaux.nodo(n,:));
    num_llamadas = num_llamadas + 1;
    % Si colisiona con el entorno
    if colision == 1 && n>0
        qnew=qaux.nodo(1:n-1,:);
        % Se cierra el bucle
        break
    elseif i==dist_max
        qnew=qaux.nodo;
    end
end
