%% link length

% REAR upper A-arm

Smi.Car.REAR.LEFT.Dimension.inner_cyl_UW      = distanza(Smi.Car.REAR.LEFT.B,Smi.Car.REAR.LEFT.UBJ);
Smi.Car.REAR.LEFT.Dimension.outer_cyl_UW      = distanza(Smi.Car.REAR.LEFT.A,Smi.Car.REAR.LEFT.UBJ);


% REAR lower A-arm

Smi.Car.REAR.LEFT.Dimension.inner_cyl_LW      = distanza(Smi.Car.REAR.LEFT.D,Smi.Car.REAR.LEFT.LBJ);
Smi.Car.REAR.LEFT.Dimension.outer_cyl_LW      = distanza(Smi.Car.REAR.LEFT.C,Smi.Car.REAR.LEFT.LBJ);

% Tie Rod

Smi.Car.REAR.LEFT.Dimension.Tie_rod           = distanza(Smi.Car.REAR.LEFT.TIE_HUB,Smi.Car.REAR.LEFT.TIE_RACK);

% Push Rod

Smi.Car.REAR.LEFT.Dimension.Pushrod           = distanza(Smi.Car.REAR.LEFT.UW_PUSHROD,Smi.Car.REAR.LEFT.CRANK_PUSHROD);

% Push Rod

Smi.Car.REAR.LEFT.Dimension.ARB_lenght        = Smi.Car.REAR.LEFT.CRANK_ARBLINK(2)-Smi.Car.REAR.LEFT.ARB_CENTER(2);

% blade

Smi.Car.REAR.LEFT.Dimension.blade_lenght      = lunghezza(Smi.Car.REAR.LEFT.ARBLINK_ARBBLADE , Smi.Car.REAR.LEFT.ARB_CENTER);

% ARB link

Smi.Car.REAR.LEFT.Dimension.ARB_link          = distanza(Smi.Car.REAR.LEFT.CRANK_ARBLINK , Smi.Car.REAR.LEFT.ARBLINK_ARBBLADE);

%% HUB PLANE

  % UBJ - LBJ
    
   Smi.Car.REAR.LEFT.Dimension.UBJ_LBJ        = distanza(Smi.Car.REAR.LEFT.UBJ , Smi.Car.REAR.LEFT.LBJ);

  % LBJ - TIE
  
   Smi.Car.REAR.LEFT.Dimension.LBJ_TIE        = distanza(Smi.Car.REAR.LEFT.LBJ , Smi.Car.REAR.LEFT.TIE_HUB);
   
  % UBJ - LBJ - TIE Plane %

   %z
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.z           = Normal_Plane(Smi.Car.REAR.LEFT.UBJ,Smi.Car.REAR.LEFT.LBJ,Smi.Car.REAR.LEFT.TIE_HUB);
   %y
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.y           = Vettore_Congiungente(Smi.Car.REAR.LEFT.UBJ,Smi.Car.REAR.LEFT.LBJ);
   %x
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.x           = cross(Smi.Car.REAR.LEFT.Dimension.plane_HUB.y , Smi.Car.REAR.LEFT.Dimension.plane_HUB.z);
   %dir vector
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.LEFT.Dimension.plane_HUB.x,Smi.Car.REAR.LEFT.Dimension.plane_HUB.y,Smi.Car.REAR.LEFT.Dimension.plane_HUB.z);
   % transormed coordinates
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.UBJ_t_coord = convert_coordinate_frame(Smi.Car.REAR.LEFT.UBJ',Smi.Car.REAR.LEFT.Dimension.plane_HUB.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_t_coord = convert_coordinate_frame(Smi.Car.REAR.LEFT.LBJ',Smi.Car.REAR.LEFT.Dimension.plane_HUB.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.TIE_t_coord = convert_coordinate_frame(Smi.Car.REAR.LEFT.TIE_HUB',Smi.Car.REAR.LEFT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.UBJ_LBJ     = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_HUB.UBJ_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_TIE     = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_HUB.TIE_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.TIE_angle   = acos(dot(Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_TIE,Smi.Car.REAR.LEFT.Dimension.plane_HUB.UBJ_LBJ)/norm(Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_TIE)*norm(Smi.Car.REAR.LEFT.Dimension.plane_HUB.UBJ_LBJ));
  
  % HUB
  
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.WC_t_coord  = convert_coordinate_frame(Smi.Car.REAR.LEFT.WHEEL_CENTER',Smi.Car.REAR.LEFT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_HUB     = Vettore_Congiungente_No_Norm(Smi.Car.REAR.LEFT.Dimension.plane_HUB.WC_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_HUB.LBJ_t_coord);
  % rotate WC to World frame
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.x           = Smi.Car.REAR.LEFT.Dimension.plane_HUB.z;
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.z           = Vettore_Congiungente(Smi.Car.REAR.LEFT.UBJ,Smi.Car.REAR.LEFT.LBJ);
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.y           = cross(Smi.Car.REAR.LEFT.Dimension.plane_HUB.z,Smi.Car.REAR.LEFT.Dimension.plane_HUB.x);
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.A           = Vector3_to_Matrix(Smi.Car.REAR.LEFT.Dimension.plane_HUB.x,Smi.Car.REAR.LEFT.Dimension.plane_HUB.y,Smi.Car.REAR.LEFT.Dimension.plane_HUB.z);
  Smi.Car.REAR.LEFT.Dimension.plane_HUB.WC_original = inv(Smi.Car.REAR.LEFT.Dimension.plane_HUB.A);
 
  
%% Elastic plane
 %z
  Smi.Car.REAR.LEFT.Dimension.z_Dir_El        = Normal_Plane(Smi.Car.REAR.LEFT.CRANK_CENTEREVOLUTE,Smi.Car.REAR.LEFT.DAMPER_CENTEREVOLUTE,Smi.Car.REAR.LEFT.UW_PUSHROD);
 %y
  Smi.Car.REAR.LEFT.Dimension.y_Dir_El        = Vettore_Congiungente(Smi.Car.REAR.LEFT.DAMPER_CENTEREVOLUTE,Smi.Car.REAR.LEFT.CRANK_CENTEREVOLUTE);
 %x
  Smi.Car.REAR.LEFT.Dimension.x_Dir_El        = cross(Smi.Car.REAR.LEFT.Dimension.y_Dir_El , Smi.Car.REAR.LEFT.Dimension.z_Dir_El);

 %rotaion matrix
  Smi.Car.REAR.LEFT.Dimension.El_Plane        = Vector3_to_Matrix(Smi.Car.REAR.LEFT.Dimension.x_Dir_El,Smi.Car.REAR.LEFT.Dimension.y_Dir_El,Smi.Car.REAR.LEFT.Dimension.z_Dir_El);


  
  
   
%% UW PLANE

   %z
  Smi.Car.REAR.LEFT.Dimension.plane_UW.z           = Normal_Plane(Smi.Car.REAR.LEFT.UBJ,Smi.Car.REAR.LEFT.A,Smi.Car.REAR.LEFT.B);
   %y
  Smi.Car.REAR.LEFT.Dimension.plane_UW.y           = Vettore_Congiungente(Smi.Car.REAR.LEFT.UBJ,Smi.Car.REAR.LEFT.A);
   %x
  Smi.Car.REAR.LEFT.Dimension.plane_UW.x           = cross(Smi.Car.REAR.LEFT.Dimension.plane_UW.y , Smi.Car.REAR.LEFT.Dimension.plane_UW.z);
   %dir vector
  Smi.Car.REAR.LEFT.Dimension.plane_UW.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.LEFT.Dimension.plane_UW.x,Smi.Car.REAR.LEFT.Dimension.plane_UW.y,Smi.Car.REAR.LEFT.Dimension.plane_UW.z);
   % transormed coordinates
  Smi.Car.REAR.LEFT.Dimension.plane_UW.A_t_coord    = convert_coordinate_frame(Smi.Car.REAR.LEFT.A',Smi.Car.REAR.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_UW.B_t_coord    = convert_coordinate_frame(Smi.Car.REAR.LEFT.B',Smi.Car.REAR.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_t_coord  = convert_coordinate_frame(Smi.Car.REAR.LEFT.UBJ',Smi.Car.REAR.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_UW.PS_t_coord   = convert_coordinate_frame(Smi.Car.REAR.LEFT.UW_PUSHROD',Smi.Car.REAR.LEFT.Dimension.plane_UW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.LEFT.Dimension.plane_UW.A_UBJ        = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_UW.A_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_B        = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_UW.B_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.LEFT.Dimension.plane_UW.wish_angle   = acos(dot(Smi.Car.REAR.LEFT.Dimension.plane_UW.A_UBJ,Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_B)...
                                                            /norm(Smi.Car.REAR.LEFT.Dimension.plane_UW.A_UBJ)*norm(Smi.Car.REAR.LEFT.Dimension.plane_UW.UBJ_B));

%% LW PLANE

   %z
  Smi.Car.REAR.LEFT.Dimension.plane_LW.z           = Normal_Plane(Smi.Car.REAR.LEFT.LBJ,Smi.Car.REAR.LEFT.C,Smi.Car.REAR.LEFT.D);
   %y
  Smi.Car.REAR.LEFT.Dimension.plane_LW.y           = Vettore_Congiungente(Smi.Car.REAR.LEFT.LBJ,Smi.Car.REAR.LEFT.C);
   %x
  Smi.Car.REAR.LEFT.Dimension.plane_LW.x           = cross(Smi.Car.REAR.LEFT.Dimension.plane_LW.y , Smi.Car.REAR.LEFT.Dimension.plane_LW.z);
   %dir vector
  Smi.Car.REAR.LEFT.Dimension.plane_LW.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.LEFT.Dimension.plane_LW.x,Smi.Car.REAR.LEFT.Dimension.plane_LW.y,Smi.Car.REAR.LEFT.Dimension.plane_LW.z);
   % transormed coordinates
  Smi.Car.REAR.LEFT.Dimension.plane_LW.C_t_coord       = convert_coordinate_frame(Smi.Car.REAR.LEFT.C',Smi.Car.REAR.LEFT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_LW.D_t_coord       = convert_coordinate_frame(Smi.Car.REAR.LEFT.D',Smi.Car.REAR.LEFT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_t_coord     = convert_coordinate_frame(Smi.Car.REAR.LEFT.LBJ',Smi.Car.REAR.LEFT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.LEFT.Dimension.plane_LW.TIE_IN_t_coord  = convert_coordinate_frame(Smi.Car.REAR.LEFT.TIE_RACK',Smi.Car.REAR.LEFT.Dimension.plane_LW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.LEFT.Dimension.plane_LW.C_LBJ        = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_LW.C_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_D        = Vettore_Congiungente(Smi.Car.REAR.LEFT.Dimension.plane_LW.D_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.LEFT.Dimension.plane_LW.wish_angle   = acos(dot(Smi.Car.REAR.LEFT.Dimension.plane_LW.C_LBJ,Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_D)...
                                                            /norm(Smi.Car.REAR.LEFT.Dimension.plane_LW.C_LBJ)*norm(Smi.Car.REAR.LEFT.Dimension.plane_LW.LBJ_D));
   % C -> TIE IN
  Smi.Car.REAR.LEFT.Dimension.plane_LW.TIE_IN_C     = Vettore_Congiungente_No_Norm(Smi.Car.REAR.LEFT.Dimension.plane_LW.TIE_IN_t_coord,Smi.Car.REAR.LEFT.Dimension.plane_LW.C_t_coord);


%% EXTRUSION PROFILES

% Crank extrusion profile   

  Crank_Extrusion_Matrix_RL;
  
% Tyre extrusion profile   

  Smi.Car.REAR.LEFT.Dimension.tyre_profile     = Tyre_extrusion(Smi.Car.REAR.LEFT.TYRE_DIAMETER,Smi.Car.REAR.LEFT.TYRE_WIDTH);
  
% OUTER ACTUATOR extrusion profile   

  Smi.Car.REAR.LEFT.Dimension.OUT_ACT_profile  = OUT_ACT_extrusion(Smi.Graphic.OUT_ACT_diam,Smi.Graphic.OUT_ACT_thick);
  
% ACTUATOR extrusion profile   

  Smi.Car.REAR.LEFT.Dimension.ACT_profile      = ACT_extrusion(Smi.Graphic.ACT_hole,Smi.Graphic.ACT_height,Smi.Graphic.ACT_diam);




