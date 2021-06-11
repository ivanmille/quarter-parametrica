%calculates the distance bewteen two points in the XZ plane
function [lung] = lunghezza(P1 , P2)

lung = sqrt( (P1(1)-P2(1))^2+(P1(3)-P2(3))^2);
lung = norm(lung);
end

