%% link length

% REAR upper A-arm

Smi.Car.REAR.RIGHT.Dimension.inner_cyl_UW      = distanza(Smi.Car.REAR.RIGHT.B,Smi.Car.REAR.RIGHT.UBJ);
Smi.Car.REAR.RIGHT.Dimension.outer_cyl_UW      = distanza(Smi.Car.REAR.RIGHT.A,Smi.Car.REAR.RIGHT.UBJ);


% REAR lower A-arm

Smi.Car.REAR.RIGHT.Dimension.inner_cyl_LW      = distanza(Smi.Car.REAR.RIGHT.D,Smi.Car.REAR.RIGHT.LBJ);
Smi.Car.REAR.RIGHT.Dimension.outer_cyl_LW      = distanza(Smi.Car.REAR.RIGHT.C,Smi.Car.REAR.RIGHT.LBJ);

% Tie Rod

Smi.Car.REAR.RIGHT.Dimension.Tie_rod           = distanza(Smi.Car.REAR.RIGHT.TIE_HUB,Smi.Car.REAR.RIGHT.TIE_RACK);

% Push Rod

Smi.Car.REAR.RIGHT.Dimension.Pushrod           = distanza(Smi.Car.REAR.RIGHT.UW_PUSHROD,Smi.Car.REAR.RIGHT.CRANK_PUSHROD);

% Push Rod

Smi.Car.REAR.RIGHT.Dimension.ARB_lenght        = norm(Smi.Car.REAR.RIGHT.CRANK_ARBLINK(2)-Smi.Car.REAR.RIGHT.ARB_CENTER(2));

% blade

Smi.Car.REAR.RIGHT.Dimension.blade_lenght      = lunghezza(Smi.Car.REAR.RIGHT.ARBLINK_ARBBLADE , Smi.Car.REAR.RIGHT.ARB_CENTER);

% ARB link

Smi.Car.REAR.RIGHT.Dimension.ARB_link          = distanza(Smi.Car.REAR.RIGHT.CRANK_ARBLINK , Smi.Car.REAR.RIGHT.ARBLINK_ARBBLADE);

%% HUB PLANE

  % UBJ - LBJ
    
   Smi.Car.REAR.RIGHT.Dimension.UBJ_LBJ        = distanza(Smi.Car.REAR.RIGHT.UBJ , Smi.Car.REAR.RIGHT.LBJ);

  % LBJ - TIE
  
   Smi.Car.REAR.RIGHT.Dimension.LBJ_TIE        = distanza(Smi.Car.REAR.RIGHT.LBJ , Smi.Car.REAR.RIGHT.TIE_HUB);
   
  % UBJ - LBJ - TIE Plane %

   %z
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z           = Normal_Plane(Smi.Car.REAR.RIGHT.UBJ,Smi.Car.REAR.RIGHT.LBJ,Smi.Car.REAR.RIGHT.TIE_HUB);
   %y
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.y           = Vettore_Congiungente(Smi.Car.REAR.RIGHT.UBJ,Smi.Car.REAR.RIGHT.LBJ);
   %x
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.x           = cross(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.y , Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z);
   %dir vector
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.x,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.y,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z);
   % transormed coordinates
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.UBJ_t_coord = convert_coordinate_frame(Smi.Car.REAR.RIGHT.UBJ',Smi.Car.REAR.RIGHT.Dimension.plane_HUB.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_t_coord = convert_coordinate_frame(Smi.Car.REAR.RIGHT.LBJ',Smi.Car.REAR.RIGHT.Dimension.plane_HUB.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.TIE_t_coord = convert_coordinate_frame(Smi.Car.REAR.RIGHT.TIE_HUB',Smi.Car.REAR.RIGHT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.UBJ_LBJ     = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.UBJ_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_TIE     = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.TIE_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.TIE_angle   = acos(dot(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_TIE,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.UBJ_LBJ)/norm(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_TIE)*norm(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.UBJ_LBJ));
  
  % HUB
  
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.WC_t_coord  = convert_coordinate_frame(Smi.Car.REAR.RIGHT.WHEEL_CENTER',Smi.Car.REAR.RIGHT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_HUB     = Vettore_Congiungente_No_Norm(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.WC_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.LBJ_t_coord);
  % rotate WC to World frame
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.x           = Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z;
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z           = Vettore_Congiungente(Smi.Car.REAR.RIGHT.UBJ,Smi.Car.REAR.RIGHT.LBJ);
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.y           = cross(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.x);
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.A           = Vector3_to_Matrix(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.x,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.y,Smi.Car.REAR.RIGHT.Dimension.plane_HUB.z);
  Smi.Car.REAR.RIGHT.Dimension.plane_HUB.WC_original = inv(Smi.Car.REAR.RIGHT.Dimension.plane_HUB.A);
 
  
%% Elastic plane
 %z
  Smi.Car.REAR.RIGHT.Dimension.z_Dir_El        = Normal_Plane(Smi.Car.REAR.RIGHT.CRANK_CENTEREVOLUTE,Smi.Car.REAR.RIGHT.DAMPER_CENTEREVOLUTE,Smi.Car.REAR.RIGHT.UW_PUSHROD);
 %y
  Smi.Car.REAR.RIGHT.Dimension.y_Dir_El        = Vettore_Congiungente(Smi.Car.REAR.RIGHT.DAMPER_CENTEREVOLUTE,Smi.Car.REAR.RIGHT.CRANK_CENTEREVOLUTE);
 %x
  Smi.Car.REAR.RIGHT.Dimension.x_Dir_El        = cross(Smi.Car.REAR.RIGHT.Dimension.y_Dir_El , Smi.Car.REAR.RIGHT.Dimension.z_Dir_El);

 %rotaion matrix
  Smi.Car.REAR.RIGHT.Dimension.El_Plane        = Vector3_to_Matrix(Smi.Car.REAR.RIGHT.Dimension.x_Dir_El,Smi.Car.REAR.RIGHT.Dimension.y_Dir_El,Smi.Car.REAR.RIGHT.Dimension.z_Dir_El);

    
%% UW PLANE

   %z
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.z           = Normal_Plane(Smi.Car.REAR.RIGHT.UBJ,Smi.Car.REAR.RIGHT.A,Smi.Car.REAR.RIGHT.B);
   %y
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.y           = Vettore_Congiungente(Smi.Car.REAR.RIGHT.UBJ,Smi.Car.REAR.RIGHT.A);
   %x
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.x           = cross(Smi.Car.REAR.RIGHT.Dimension.plane_UW.y , Smi.Car.REAR.RIGHT.Dimension.plane_UW.z);
   %dir vector
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.RIGHT.Dimension.plane_UW.x,Smi.Car.REAR.RIGHT.Dimension.plane_UW.y,Smi.Car.REAR.RIGHT.Dimension.plane_UW.z);
   % transormed coordinates
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.A_t_coord    = convert_coordinate_frame(Smi.Car.REAR.RIGHT.A',Smi.Car.REAR.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.B_t_coord    = convert_coordinate_frame(Smi.Car.REAR.RIGHT.B',Smi.Car.REAR.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_t_coord  = convert_coordinate_frame(Smi.Car.REAR.RIGHT.UBJ',Smi.Car.REAR.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.PS_t_coord   = convert_coordinate_frame(Smi.Car.REAR.RIGHT.UW_PUSHROD',Smi.Car.REAR.RIGHT.Dimension.plane_UW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.A_UBJ        = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_UW.A_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_B        = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_UW.B_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_UW.wish_angle   = acos(dot(Smi.Car.REAR.RIGHT.Dimension.plane_UW.A_UBJ,Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_B)...
                                                            /norm(Smi.Car.REAR.RIGHT.Dimension.plane_UW.A_UBJ)*norm(Smi.Car.REAR.RIGHT.Dimension.plane_UW.UBJ_B));

                                                        
 %% LW PLANE

   %z
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.z           = Normal_Plane(Smi.Car.REAR.RIGHT.LBJ,Smi.Car.REAR.RIGHT.C,Smi.Car.REAR.RIGHT.D);
   %y
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.y           = Vettore_Congiungente(Smi.Car.REAR.RIGHT.LBJ,Smi.Car.REAR.RIGHT.C);
   %x
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.x           = cross(Smi.Car.REAR.RIGHT.Dimension.plane_LW.y , Smi.Car.REAR.RIGHT.Dimension.plane_LW.z);
   %dir vector
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.dir_vec     = Vector3_to_Matrix(Smi.Car.REAR.RIGHT.Dimension.plane_LW.x,Smi.Car.REAR.RIGHT.Dimension.plane_LW.y,Smi.Car.REAR.RIGHT.Dimension.plane_LW.z);
   % transormed coordinates
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_t_coord       = convert_coordinate_frame(Smi.Car.REAR.RIGHT.C',Smi.Car.REAR.RIGHT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.D_t_coord       = convert_coordinate_frame(Smi.Car.REAR.RIGHT.D',Smi.Car.REAR.RIGHT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_t_coord     = convert_coordinate_frame(Smi.Car.REAR.RIGHT.LBJ',Smi.Car.REAR.RIGHT.Dimension.plane_LW.dir_vec);
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.TIE_IN_t_coord  = convert_coordinate_frame(Smi.Car.REAR.RIGHT.TIE_RACK',Smi.Car.REAR.RIGHT.Dimension.plane_LW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_LBJ        = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_t_coord);
   % LBJ -> TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_D        = Vettore_Congiungente(Smi.Car.REAR.RIGHT.Dimension.plane_LW.D_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.wish_angle   = acos(dot(Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_LBJ,Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_D)...
                                                            /norm(Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_LBJ)*norm(Smi.Car.REAR.RIGHT.Dimension.plane_LW.LBJ_D));
   % C -> TIE IN
  Smi.Car.REAR.RIGHT.Dimension.plane_LW.TIE_IN_C        = Vettore_Congiungente_No_Norm(Smi.Car.REAR.RIGHT.Dimension.plane_LW.TIE_IN_t_coord,Smi.Car.REAR.RIGHT.Dimension.plane_LW.C_t_coord);


%% EXTRUSION PROFILES
% Crank extrusion profile   

  Crank_Extrusion_Matrix_RR;
  
% Tyre extrusion profile   

  Smi.Car.REAR.RIGHT.Dimension.tyre_profile     = Tyre_extrusion(Smi.Car.REAR.RIGHT.TYRE_DIAMETER,Smi.Car.REAR.RIGHT.TYRE_WIDTH);
  
% OUTER ACTUATOR extrusion profile   

  Smi.Car.REAR.RIGHT.Dimension.OUT_ACT_profile  = OUT_ACT_extrusion(Smi.Graphic.OUT_ACT_diam,Smi.Graphic.OUT_ACT_thick);
  
% ACTUATOR extrusion profile   

  Smi.Car.REAR.RIGHT.Dimension.ACT_profile      = ACT_extrusion(Smi.Graphic.ACT_hole,Smi.Graphic.ACT_height,Smi.Graphic.ACT_diam);
 
