%find a vector perpendicolar to the plane containing P1, P2 and P3
function [normal] = Normal_Plane(P1, P2, P3)

normal = cross(P2-P1, P3-P1);

normal = normal/norm(normal);

end

