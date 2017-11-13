function [nodos_A_XYZ,nodos_B_XYZ] = dibujarArboles(arbolA,arbolB,qinicial,qobjetivo)
%% Se definen variables:
    n_nodos_A = size(arbolA.nodos,1);
    nodos_A_XYZ = zeros(n_nodos_A,6);

    n_nodos_B = size(arbolB.nodos,1);
    nodos_B_XYZ = zeros(n_nodos_B,6);
%% Se convierten los nodos de articulares al espacio XYZ:
    % Nodos del arbol A:
    for i=1:n_nodos_A
        apoloSetRobotJoints('ASEA',arbolA.nodos(i,:));
        nodos_A_XYZ(i,:) = apoloGetLocation('pieza');
    end
    % Nodos del arbol B:
    for i=1:n_nodos_B
        apoloSetRobotJoints('ASEA',arbolB.nodos(i,:));
        nodos_B_XYZ(i,:) = apoloGetLocation('pieza');
    end

    % Se añaden el nodo inicial y final
    apoloSetRobotJoints('ASEA',qinicial);   % nodo inicial
    nodo_inicial = apoloGetLocation('pieza');
    apoloSetRobotJoints('ASEA',qobjetivo);  % nodo final
    nodo_final = apoloGetLocation('pieza');

%% Se pintan los arboles
    figure;
    hold on;
    grid on;
    % Se pintan los nodos del arbol A:
        scatter3(nodos_A_XYZ(1:end,1),nodos_A_XYZ(1:end,2),nodos_A_XYZ(1:end,3),'.','r')
        % Se une cada nodo mediante la lista de aristas
        for i=1:n_nodos_A
            for j=1:n_nodos_A
                if arbolA.aristas(i,j)==1 
                    line([nodos_A_XYZ(i,1) nodos_A_XYZ(j,1)],...
                         [nodos_A_XYZ(i,2) nodos_A_XYZ(j,2)],...
                         [nodos_A_XYZ(i,3) nodos_A_XYZ(j,3)],'Color','k');
                end
            end
        end
    
    % Se pintan los nodos del arbol B:
        scatter3(nodos_B_XYZ(1:end,1),nodos_B_XYZ(1:end,2),nodos_B_XYZ(1:end,3),'.','b')
        % Se une cada nodo mediante la lista de aristas
        for i=1:n_nodos_B
            for j=1:n_nodos_B
                if arbolB.aristas(i,j)
                    line([nodos_B_XYZ(i,1) nodos_B_XYZ(j,1)],...
                         [nodos_B_XYZ(i,2) nodos_B_XYZ(j,2)],...
                         [nodos_B_XYZ(i,3) nodos_B_XYZ(j,3)],'Color','g');
                end
            end
        end

    % Se pintan los nodos inicial y final
    scatter3(nodo_inicial(1),nodo_inicial(2),nodo_inicial(3),...
            'MarkerEdgeColor','k','SizeData',50,'MarkerFaceColor','g','SizeData',100)
    scatter3(nodo_final(1),nodo_final(2),nodo_final(3),...
            'MarkerEdgeColor','k','SizeData',50,'MarkerFaceColor','c','SizeData',100)
        
        
    % Se calcula el nodo de union de los arboles
    apoloSetRobotJoints('ASEA',arbolA.nodos(end,:));
    nodo_union = apoloGetLocation('pieza');
    
    scatter3(nodo_union(1),nodo_union(2) ,nodo_union(3),...
            'MarkerEdgeColor','k','SizeData',50,'MarkerFaceColor','y','SizeData',100)
   
    hold off;

    title(['Arbol A: ',num2str(n_nodos_A),' nodos / Arbol B: ',num2str(n_nodos_B),' nodos']);

    xlabel(' X (m)');
    ylabel(' Y (m)');
    zlabel(' Z (m)');

    hold off;

    view([25 25]);

