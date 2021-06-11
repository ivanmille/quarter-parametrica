%% link length

% front upper A-arm

Smi.Car.FRONT.RIGHT.Dimension.inner_cyl_UW      = distanza(Smi.Car.FRONT.RIGHT.B,Smi.Car.FRONT.RIGHT.UBJ);
Smi.Car.FRONT.RIGHT.Dimension.outer_cyl_UW      = distanza(Smi.Car.FRONT.RIGHT.A,Smi.Car.FRONT.RIGHT.UBJ);


% front lower A-arm

Smi.Car.FRONT.RIGHT.Dimension.inner_cyl_LW      = distanza(Smi.Car.FRONT.RIGHT.D,Smi.Car.FRONT.RIGHT.LBJ);
Smi.Car.FRONT.RIGHT.Dimension.outer_cyl_LW      = distanza(Smi.Car.FRONT.RIGHT.C,Smi.Car.FRONT.RIGHT.LBJ);

% Steering Rack

Smi.Car.FRONT.RIGHT.Dimension.steering_rack     = distanza(Smi.Car.FRONT.RIGHT.RACK_CENTER,Smi.Car.FRONT.RIGHT.TIE_RACK);

% Tie Rod

Smi.Car.FRONT.RIGHT.Dimension.Tie_rod           = distanza(Smi.Car.FRONT.RIGHT.TIE_HUB,Smi.Car.FRONT.RIGHT.TIE_RACK);

% Push Rod

Smi.Car.FRONT.RIGHT.Dimension.Pushrod           = distanza(Smi.Car.FRONT.RIGHT.UW_PUSHROD,Smi.Car.FRONT.RIGHT.CRANK_PUSHROD);

% Push Rod

Smi.Car.FRONT.RIGHT.Dimension.ARB_lenght        = Smi.Car.FRONT.RIGHT.CRANK_ARBLINK(2)-Smi.Car.FRONT.RIGHT.ARB_CENTER(2);

% blade

Smi.Car.FRONT.RIGHT.Dimension.blade_lenght      = lunghezza(Smi.Car.FRONT.RIGHT.ARBLINK_ARBBLADE , Smi.Car.FRONT.RIGHT.ARB_CENTER);

% ARB link

Smi.Car.FRONT.RIGHT.Dimension.ARB_link          = distanza(Smi.Car.FRONT.RIGHT.CRANK_ARBLINK , Smi.Car.FRONT.RIGHT.ARBLINK_ARBBLADE);

%% HUB PLANE

  % UBJ - LBJ
    
   Smi.Car.FRONT.RIGHT.Dimension.UBJ_LBJ        = distanza(Smi.Car.FRONT.RIGHT.UBJ , Smi.Car.FRONT.RIGHT.LBJ);

  % LBJ - TIE
  
   Smi.Car.FRONT.RIGHT.Dimension.LBJ_TIE        = distanza(Smi.Car.FRONT.RIGHT.LBJ , Smi.Car.FRONT.RIGHT.TIE_HUB);
   
  % UBJ - LBJ - TIE Plane %

   %z
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z           = Normal_Plane(Smi.Car.FRONT.RIGHT.UBJ,Smi.Car.FRONT.RIGHT.LBJ,Smi.Car.FRONT.RIGHT.TIE_HUB);
   %y
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.y           = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.UBJ,Smi.Car.FRONT.RIGHT.LBJ);
   %x
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.x           = cross(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.y , Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z);
   %dir vector
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.x,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.y,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z);
   % transormed coordinates
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.UBJ_t_coord = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.UBJ',Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_t_coord = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.LBJ',Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.TIE_t_coord = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.TIE_HUB',Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.UBJ_LBJ     = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.UBJ_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_TIE     = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.TIE_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.TIE_angle   = acos(dot(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_TIE,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.UBJ_LBJ)/norm(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_TIE)*norm(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.UBJ_LBJ));
  
  
  % HUB
  
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.WC_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.WHEEL_CENTER',Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_HUB     = Vettore_Congiungente_No_Norm(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.WC_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.LBJ_t_coord);
  % rotate WC to World frame
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.x           = Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z;
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z           = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.UBJ,Smi.Car.FRONT.RIGHT.LBJ);
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.y           = cross(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.x);
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.A           = Vector3_to_Matrix(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.x,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.y,Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.z);
  Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.WC_original = inv(Smi.Car.FRONT.RIGHT.Dimension.plane_HUB.A);
  
  
%% Elastic plane
 %z
  Smi.Car.FRONT.RIGHT.Dimension.z_Dir_El        = Normal_Plane(Smi.Car.FRONT.RIGHT.CRANK_CENTEREVOLUTE,Smi.Car.FRONT.RIGHT.DAMPER_CENTEREVOLUTE,Smi.Car.FRONT.RIGHT.UW_PUSHROD);
 %y
  Smi.Car.FRONT.RIGHT.Dimension.y_Dir_El        = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.DAMPER_CENTEREVOLUTE,Smi.Car.FRONT.RIGHT.CRANK_CENTEREVOLUTE);
 %x
  Smi.Car.FRONT.RIGHT.Dimension.x_Dir_El        = cross(Smi.Car.FRONT.RIGHT.Dimension.y_Dir_El , Smi.Car.FRONT.RIGHT.Dimension.z_Dir_El);

 %rotaion matrix
  Smi.Car.FRONT.RIGHT.Dimension.El_Plane        = Vector3_to_Matrix(Smi.Car.FRONT.RIGHT.Dimension.x_Dir_El,Smi.Car.FRONT.RIGHT.Dimension.y_Dir_El,Smi.Car.FRONT.RIGHT.Dimension.z_Dir_El);

  

  
   
 %% UW PLANE

   %z
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.z           = Normal_Plane(Smi.Car.FRONT.RIGHT.UBJ,Smi.Car.FRONT.RIGHT.A,Smi.Car.FRONT.RIGHT.B);
   %y
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.y           = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.UBJ,Smi.Car.FRONT.RIGHT.A);
   %x
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.x           = cross(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.y , Smi.Car.FRONT.RIGHT.Dimension.plane_UW.z);
   %dir vector
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.x,Smi.Car.FRONT.RIGHT.Dimension.plane_UW.y,Smi.Car.FRONT.RIGHT.Dimension.plane_UW.z);
   % transormed coordinates
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.A_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.A',Smi.Car.FRONT.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.B_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.B',Smi.Car.FRONT.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.UBJ',Smi.Car.FRONT.RIGHT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.PS_t_coord   = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.UW_PUSHROD',Smi.Car.FRONT.RIGHT.Dimension.plane_UW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.A_UBJ        = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_UW.A_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_B        = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.B_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_UW.wish_angle   = acos(dot(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.A_UBJ,Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_B)...
                                                            /norm(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.A_UBJ)*norm(Smi.Car.FRONT.RIGHT.Dimension.plane_UW.UBJ_B));

 %% LW Plane

   %z
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.z           = Normal_Plane(Smi.Car.FRONT.RIGHT.LBJ,Smi.Car.FRONT.RIGHT.C,Smi.Car.FRONT.RIGHT.D);
   %y
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.y           = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.LBJ,Smi.Car.FRONT.RIGHT.C);
   %x
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.x           = cross(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.y , Smi.Car.FRONT.RIGHT.Dimension.plane_LW.z);
   %dir vector
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.x,Smi.Car.FRONT.RIGHT.Dimension.plane_LW.y,Smi.Car.FRONT.RIGHT.Dimension.plane_LW.z);
   % transormed coordinates
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.C_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.C',Smi.Car.FRONT.RIGHT.Dimension.plane_LW.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.D_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.D',Smi.Car.FRONT.RIGHT.Dimension.plane_LW.dir_vec);
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.RIGHT.LBJ',Smi.Car.FRONT.RIGHT.Dimension.plane_LW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.C_LBJ        = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_LW.C_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_D        = Vettore_Congiungente(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.D_t_coord,Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.RIGHT.Dimension.plane_LW.wish_angle   = acos(dot(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.C_LBJ,Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_D)...
                                                            /norm(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.C_LBJ)*norm(Smi.Car.FRONT.RIGHT.Dimension.plane_LW.LBJ_D));

                                                        
%%  EXTRISION PROFILES

% Crank extrusion profile   

  Crank_Extrusion_Matrix_FR;
  
% Tyre extrusion profile   

  Smi.Car.FRONT.RIGHT.Dimension.tyre_profile     = Tyre_extrusion(Smi.Car.FRONT.RIGHT.TYRE_DIAMETER,Smi.Car.FRONT.RIGHT.TYRE_WIDTH);
  
% OUTER ACTUATOR extrusion profile   

  Smi.Car.FRONT.RIGHT.Dimension.OUT_ACT_profile  = OUT_ACT_extrusion(Smi.Graphic.OUT_ACT_diam,Smi.Graphic.OUT_ACT_thick);
  
% ACTUATOR extrusion profile   

  Smi.Car.FRONT.RIGHT.Dimension.ACT_profile      = ACT_extrusion(Smi.Graphic.ACT_hole,Smi.Graphic.ACT_height,Smi.Graphic.ACT_diam);
  
