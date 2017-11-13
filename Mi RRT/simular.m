function simular (ruta)
        % Simular
        for i = 1:size(ruta,1)
            apoloSetRobotJoints('ASEA', ruta(i,:));
            apoloUpdate;
            pause(0.2);
        end


