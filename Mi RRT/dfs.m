function path = dfs( mtree, ktarg )

    %% Parámetros
    knum = size(mtree,1);       % Número de nodos
    state = zeros(1,knum);      % Vector de estados 
                                % {0=no-visitado, 1=abierto, 2=cerrado}
    path = [];                  % Trayectoria

    %% Algoritmo
    % Para cada nodo
    for k = 1:knum
        % Si no se ha visitado
        if state(k) == 0
            % Aplicar la función visitar
            [state, found, path] = dfs_visit(k, state, mtree, knum, ktarg, path);
        end      
        % Si se ha encontrado el nodo objetivo salir del bucle
        if found == 1
            break
        end
    end

end

function [state, found, path] = dfs_visit(k, state, mtree, knum, ktarg, path)

    % Marcar el nodo actual como abierto
    state(k) = 1;
    % Agregar el nodo actual a la trayectoria
    path(end+1) = k;
    
    % Si el nodo actual es el nodo objetivo, indicarlo y salir
    if k == ktarg
        found = 1;
        return
    else
        found = 0;
    end

    % Buscar para cada nodo
    for i = 1:knum
        % Si es vecino del actual
        if mtree(k,i)==1
            % Y si no se ha visitado
            if state(i)==0
                % Visitarlo
                [state, found, path] = dfs_visit(i, state, mtree, knum, ktarg, path);
                % Si se ha encontrado el nodo objetivo, salir
                if found == 1
                    return
                end
            end
        end
    end

    % Marcar el nodo actual como cerrado
    state(k) = 2;
    % Extraer el nodo actual de la trayectoria
    path(end) = [];

end