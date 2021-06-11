% Vector containing the points necessary for extruding the outer actuator
function [vector] = OUT_ACT_extrusion(diameter,thickness)

vector = [diameter/2-10 -thickness/2 ;...
          diameter/2 -thickness/2 ;...
          diameter/2 thickness/2 ;...
          diameter/2-10 thickness/2];

end


