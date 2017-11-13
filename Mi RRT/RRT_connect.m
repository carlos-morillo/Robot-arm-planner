
%addpath('C:\Program Files (x86)\Apolo\Matlab')

function RRT_connect
addpath('C:\Program Files (x86)\Apolo\Matlab')
clear
close all
clc

%% Definición de variables
    % Espacio de trabajo de las articulaciones                                     
    qmax = [ 0.7  1.75  1.05  3.49  2.09  3.49]; 
    
    % Posiciones inicial y final del robot
    qinicial = [0 1.43 -0.88 0 0 0];    % Posicion inicial
    qobjetivo = [0 0.94 -0.88 0 0 0];   % Posicion final 
    
    % Posicion aleatoria del robot
    qrand = zeros(1,6);                 % Posicion aleatoria

    % Se definen las variables de los dos arboles en una estructura
    arbol_A=struct('nodos',qinicial,'aristas',0);
    arbol_B=struct('nodos',qobjetivo,'aristas',0);
    arbol=struct('expansion',arbol_B,'busqueda',arbol_A);

    % Otros
    paso = pi/20;   % Paso de análisis hasta el siguiente nodo
    imax = 3000;    % Máximo número de iteraciones                            
    nodos_rama=1;   % Distancia de seleccion de nodos de la rama creada
    
%% Comienzo del algoritmo 
    % Contador de tiempo
    tic

    % Contador de llamadas al colisionador
    llamadas_totales = 0;

    % Bucle RRT
    for i = 1:imax
        % Indicador de iteracion:
        fprintf('Iteración(%d)\n', i)  

        % Al principio de cada iteración se intercambian los arboles de
        % busqueda por el de expansion y viceversa:
        arbol=struct('expansion',arbol.busqueda,'busqueda',arbol.expansion);

        % Generacion del nodo aleatorio:
        for j=1:size(qmax,2)
            qrand(j) = qmax(j)*(2*rand(1)-1);
        end

    %%%%%%%%%%%%%%%%%%%%%
    %%%% 1.Expansion %%%%
    %%%%%%%%%%%%%%%%%%%%%
    % El arbol.expansion se extiende hacia el nodo aleatorio
            
            % 1.1. 
            % Se busca el nodo del arbol.expansion mas cercano al nodo
            % aleatorio:
            distancia_minima = norm(qrand-arbol.expansion.nodos(1,:));
            qnear = arbol.expansion.nodos(1,:);
            indice_nodo=1;
            for nodo=1:size(arbol.expansion.nodos,1)
                distancia = norm(qrand-arbol.expansion.nodos(nodo,:));
                if distancia < distancia_minima
                    indice_nodo = nodo;
                    distancia_minima = distancia;
                    qnear = arbol.expansion.nodos(nodo,:);
                end
            end
            
            % 1.2.
            % Desde el nodo mas cercano (1.1) hasta el nodo aleatorio
            % se genera una rama de nodos (qnew) que no presenta colision
            % con el espacio ocupado
            [qnew,num_llamadas] = detectarColision(qnear,qrand,qmax,paso);
            llamadas_totales = llamadas_totales + num_llamadas;
            
            % 1.3.
            % La rama sin colision obtenida en (1.2) se añade al
            % arbol.expansion
            %   - nodos_rama: controla la cantidad de nodos de la rama que
            %                se van a añadir. Evita la sobrecarga del grafo
            for nodo=1:nodos_rama:size(qnew,1)
                % Guardar el nodo en el árbol
                arbol.expansion.nodos(end+1,:) = qnew(nodo,:);
                % Se crea la nueva arista entre nodos
                % - Para el primer nodo la arista se une con el arbol
                if nodo==1
                    arbol.expansion.aristas(indice_nodo,end+1) = 1;
                    arbol.expansion.aristas(end+1,indice_nodo) = 1;
                    % - Para otros nodos, las aristas se crean formando la rama
                    %   completa
                else
                    arbol.expansion.aristas(end,end+1) = 1;
                    arbol.expansion.aristas(end+1,end-1) = 1;
                end
            end
            
    %%%%%%%%%%%%%%%%%%%%
    %%%% 2.Conexion %%%%
    %%%%%%%%%%%%%%%%%%%%
    % El arbol.busqueda se extiende hacia el nodo añadido al
    % arbol.expansion
            
            % 2.1.
            % Se selecciona el ultimo nodo añadido a arbol.expasion
            qnew_arbol_exp = qnew(end,:);
            
            % 2.2.
            % Se busca el nodo del arbol.busqueda mas cercano al nodo
            % qnew_expansion:
            distancia_minima = norm(qnew_arbol_exp-arbol.busqueda.nodos(1,:));
            qnear = arbol.busqueda.nodos(1,:);
            indice_nodo=1;
            for nodo=1:size(arbol.busqueda.nodos,1)
                distancia = norm(qnew_arbol_exp-arbol.busqueda.nodos(nodo,:));
                if distancia < distancia_minima
                    qnear = arbol.busqueda.nodos(nodo,:);
                    indice_nodo = nodo;
                    distancia_minima = distancia;
                end
            end
            
            % 2.3.
            % Desde el nodo mas cercano (2.2) hasta el nodo del
            % arbol.expansion se genera una rama de nodos (qnew_arbol_busq)
            % que no presenta colision con el espacio ocupado
            [qnew_arbol_busq,num_llamadas] = detectarColision(qnear,qnew_arbol_exp,qmax,paso);
            llamadas_totales = llamadas_totales + num_llamadas;
            
            % 2.4.
            % La rama sin colision obtenida en (2.3) se añade al
            % arbol.busqueda
            %   - nodos_rama: controla la cantidad de nodos de la rama que
            %                se van a añadir. Evita la sobrecarga del grafo
            for nodo=1:nodos_rama:size(qnew_arbol_busq,1)
                % Guardar el nodo en el árbol
                arbol.busqueda.nodos(end+1,:) = qnew_arbol_busq(nodo,:);
                % Se crea la nueva arista entre nodos
                % - Para el primer nodo la arista se une con el arbol
                if nodo==1
                    arbol.busqueda.aristas(indice_nodo,end+1) = 1;
                    arbol.busqueda.aristas(end+1,indice_nodo) = 1;
                % - Para otros nodos, las aristas se crean formando la rama
                %   completa
                else
                    arbol.busqueda.aristas(end,end+1) = 1;
                    arbol.busqueda.aristas(end+1,end-1) = 1;
                end
            end
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 3.Estudio de conectividad %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compara los ultimos nodos añadidos al grafo, si son iguales el habrá
    % conexion entre los dos arboles
            
            % 3.1. 
            % Si hay conexion: sale del bucle
            if qnew_arbol_busq(end,:) == qnew_arbol_exp;
                conectado=1;
                nodo_union_exp=size(arbol.expansion.nodos,1);
                nodo_union_busq=size(arbol.busqueda.nodos,1);
                break
            % 3.2.
            % Si no hay conexion: continua el bucle
            else
                conectado=0;
            end
    end

    % Se detiene el contador de tiempo
    tiempo=toc;
    % Se muestra el mensaje de finalizacion
    fprintf('¡Ruta encontrada!\n');

%% Busqueda de la ruta por DFS

        % Se inicia el contrador para el DFS
        tic

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 1.Ruta del arbol A %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        % Se obtienen los indices de los nodos del arbol A de la ruta que
        % conecta el nodo inicial con el nodo de union entre arboles
        ruta_A = dfs(arbol.expansion.aristas, nodo_union_exp);  
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 2.Ruta del arbol B %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Se obtienen los indices de los nodos del arbol B de la ruta que
        % conecta el nodo inicial con el nodo de union entre arboles
        ruta_B = dfs( arbol.busqueda.aristas, nodo_union_busq);

    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 2.Ruta completa %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
        % Se unifican las trayectorias anteriores para formar el camino
        % completo, se cogen los nodos de cada ruta y se concatenan en el
        % nuevo vector
        nodos_ruta_A = size(ruta_A,2);
        nodos_ruta_B = size(ruta_B,2);
        ruta_final = zeros(nodos_ruta_A+nodos_ruta_B, size(qmax,2));
        for i = 1:nodos_ruta_A
            nodo = ruta_A(i);
            ruta_final(i,:) = arbol.expansion.nodos(nodo,:);
        end
        for i = 1:nodos_ruta_B
            nodo = ruta_B(i);
            ruta_final(end+1-i,:) = arbol.busqueda.nodos(nodo,:);
        end
        
        j=0;
        for i=1:6
            if ruta_final(end,i) == qinicial(i)
                j=i;
            end
            if j==6
                ruta_final = ruta_final(end:-1:1,:);
            end
        end
        % Detener el contador de tiempo
        tiempo2=toc;
       
        
%% Se pintan los arboles y la ruta

    % Se pintan los arboles generados
    dibujarArboles(arbol.expansion,arbol.busqueda,qinicial,qobjetivo);
    
    % Se pinta la ruta
    dibujarRuta(ruta_final);
        
%% Resultados

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 1.Resultados RRT %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%

        % Si los dos arboles estan conectados:
        if conectado
            fprintf('\nDATOS DEL RRT:\n')
            fprintf('- Nodos totales generados = %d\n',size(arbol.expansion.nodos,1)+size(arbol.busqueda.nodos,1))
            fprintf('- Llamadas al colisionador = %d\n', llamadas_totales)
            fprintf('- Tiempo total RRT = %.3fs\n\n', tiempo)
            % Si no se ha encontrado conexión entre los árboles
        else
            fprintf('\nSUPERADO TIEMPO LIMITE\n')
            % Se sale del programa
            return
        end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% 2.Resultados DFS %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
        % Se muestran los datos de la ruta
        fprintf('\nDATOS DE LA RUTA\n')
        fprintf('- Nodos totales de la ruta = %d\n', size(ruta_final,1))
        fprintf('- Tiempo total DFS = %.3fs\n', tiempo2)
        % Se pregunta si se desea ejecutar la simulacion
        simulacion=input('\n¿Desea ejecutar la simulacion? (1/0)\n');


%% Simular el movimiento del robot


        while simulacion
            simular(ruta_final)
            simulacion=input('\n¿Desea ejecutar la simulacion? (1/0)\n');
        end

    