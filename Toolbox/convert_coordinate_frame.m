% given a vector and a new UCS calculates the new coordinates
function [Q] = convert_coordinate_frame(v,A)

Q=A\v;

end

