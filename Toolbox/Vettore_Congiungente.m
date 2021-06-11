%find the versor connecting two points
function [vector] = Vettore_Congiungente(P1, P2)

vector = P1-P2;

vector = vector / norm(vector) ;

end


