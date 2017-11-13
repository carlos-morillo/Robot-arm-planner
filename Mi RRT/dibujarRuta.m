
function dibujarRuta(ruta)

    ruta_XYZ=zeros(size(ruta,1),6);
    for i=1:size(ruta,1)
        apoloSetRobotJoints('ASEA',ruta(i,:));
        ruta_XYZ(i,:) = apoloGetLocation('pieza');
    end
    figure;
    hold on
    grid on
    scatter3(ruta_XYZ(1:end,1),ruta_XYZ(1:end,2),ruta_XYZ(1:end,3),'k')
    % Se une cada nodo mediante la lista de aristas
    for i=1:size(ruta,1)-1
        line([ruta_XYZ(i,1) ruta_XYZ(i+1,1)],...
            [ruta_XYZ(i,2) ruta_XYZ(i+1,2)],...
            [ruta_XYZ(i,3) ruta_XYZ(i+1,3)],'Color','r');  
    end
    title('Ruta del robot / Nodos = ');
    size(ruta)
    xlabel(' X (m)');
    ylabel(' Y (m)');
    zlabel(' Z (m)');

    hold off

    view([25 25])