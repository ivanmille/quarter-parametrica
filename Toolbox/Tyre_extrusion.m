% Vector containing the points necessary for extruding the tyre
function [vector] = Tyre_extrusion(diameter,width)

vector = [diameter/2-10 -width/2 ; diameter/2 -width/2 ; diameter/2 width/2 ; diameter/2-10 width/2];

end


