% Vector containing the points necessary for extruding the crank

Trans_matrix=Smi.Car.FRONT.RIGHT.Dimension.El_Plane;

a=Trans_matrix\Smi.Car.FRONT.RIGHT.CRANK_CENTEREVOLUTE';
b=Trans_matrix\Smi.Car.FRONT.RIGHT.CRANK_PUSHROD';
c=Trans_matrix\Smi.Car.FRONT.RIGHT.CRANK_STEM';
d=Trans_matrix\Smi.Car.FRONT.RIGHT.CRANK_ARBLINK';

Smi.Car.FRONT.RIGHT.Dimension.Crank_ext       = [0 0; d(1)-b(1) d(2)-b(2) ; c(1)-b(1) c(2)-b(2) ; a(1)-b(1) a(2)-b(2)] ;
Smi.Car.FRONT.RIGHT.crank.CENTER_PUSH         = [a(1)-b(1) a(2)-b(2) 0];
Smi.Car.FRONT.RIGHT.crank.ARB_PUSH            = [d(1)-b(1) d(2)-b(2) 0];
Smi.Car.FRONT.RIGHT.crank.STEM_PUSH           = [c(1)-b(1) c(2)-b(2) 0];

clear a b c d Trans_matrix