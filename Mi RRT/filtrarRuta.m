function [ruta_optimizada]=filtrarRuta(ruta,qmax,paso)

for n=0:8
    total_nodos_ruta=size(ruta,1);
    n
    nodo_A = round((total_nodos_ruta-2)*rand(1));
    if nodo_A==0
        nodo_A=1;
    end
    nodo_B = nodo_A+2;
%     nodo_B = round(total_nodos_ruta*rand(1));
%     if nodo_A==nodo_B
%         nodo_B=nodo_A+1;
%     end
%     
    [qnew,~] = detectarColision(ruta(nodo_A,:),ruta(nodo_B,:),qmax,paso);

    tramo_final=zeros(total_nodos_ruta-nodo_B,6);
   % if qnew(end) == ruta(nodo_B,:)
        tam_tramo_nuevo=size(qnew,1);
        for i=nodo_B:total_nodos_ruta
            tramo_final(i,:)=ruta(i,:);
        end
        for j=nodo_A:tam_tramo_nuevo
            ruta(j,:)=qnew(j,:);
        end
        for k=1:size(tramo_final,1)
            aux(k,:)=tramo_final(k,:);
        end
        
        ruta=[ruta;tramo_final];
    %end
end
ruta_optimizada=ruta;