%creates a matrix starting from 3 vectors
function [M] = Vector3_to_Matrix(v1,v2,v3)

M= [v1(1) v2(1) v3(1);...
    v1(2) v2(2) v3(2);...
    v1(3) v2(3) v3(3)];
end

