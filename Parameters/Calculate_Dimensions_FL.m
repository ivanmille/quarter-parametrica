%% link length

% front upper A-arm

Smi.Car.FRONT.LEFT.Dimension.inner_cyl_UW      = distanza(Smi.Car.FRONT.LEFT.B,Smi.Car.FRONT.LEFT.UBJ);
Smi.Car.FRONT.LEFT.Dimension.outer_cyl_UW      = distanza(Smi.Car.FRONT.LEFT.A,Smi.Car.FRONT.LEFT.UBJ);


% front lower A-arm

Smi.Car.FRONT.LEFT.Dimension.inner_cyl_LW      = distanza(Smi.Car.FRONT.LEFT.D,Smi.Car.FRONT.LEFT.LBJ);
Smi.Car.FRONT.LEFT.Dimension.outer_cyl_LW      = distanza(Smi.Car.FRONT.LEFT.C,Smi.Car.FRONT.LEFT.LBJ);

% Steering Rack

Smi.Car.FRONT.LEFT.Dimension.steering_rack     = distanza(Smi.Car.FRONT.LEFT.RACK_CENTER,Smi.Car.FRONT.LEFT.TIE_RACK);

% Tie Rod

Smi.Car.FRONT.LEFT.Dimension.Tie_rod           = distanza(Smi.Car.FRONT.LEFT.TIE_HUB,Smi.Car.FRONT.LEFT.TIE_RACK);

% Push Rod

Smi.Car.FRONT.LEFT.Dimension.Pushrod           = distanza(Smi.Car.FRONT.LEFT.UW_PUSHROD,Smi.Car.FRONT.LEFT.CRANK_PUSHROD);

% Push Rod

Smi.Car.FRONT.LEFT.Dimension.ARB_lenght        = Smi.Car.FRONT.LEFT.CRANK_ARBLINK(2)-Smi.Car.FRONT.LEFT.ARB_CENTER(2);

% blade

Smi.Car.FRONT.LEFT.Dimension.blade_lenght      = lunghezza(Smi.Car.FRONT.LEFT.ARBLINK_ARBBLADE , Smi.Car.FRONT.LEFT.ARB_CENTER);

% ARB link

Smi.Car.FRONT.LEFT.Dimension.ARB_link          = distanza(Smi.Car.FRONT.LEFT.CRANK_ARBLINK , Smi.Car.FRONT.LEFT.ARBLINK_ARBBLADE);

%% HUB plane
 
  % UBJ - LBJ
    
   Smi.Car.FRONT.LEFT.Dimension.UBJ_LBJ        = distanza(Smi.Car.FRONT.LEFT.UBJ , Smi.Car.FRONT.LEFT.LBJ);

  % LBJ - TIE
  
   Smi.Car.FRONT.LEFT.Dimension.LBJ_TIE        = distanza(Smi.Car.FRONT.LEFT.LBJ , Smi.Car.FRONT.LEFT.TIE_HUB);
   
  % UBJ - LBJ - TIE Plane %

   %z
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z           = Normal_Plane(Smi.Car.FRONT.LEFT.UBJ,Smi.Car.FRONT.LEFT.LBJ,Smi.Car.FRONT.LEFT.TIE_HUB);
   %y
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.y           = Vettore_Congiungente(Smi.Car.FRONT.LEFT.UBJ,Smi.Car.FRONT.LEFT.LBJ);
   %x
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.x           = cross(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.y , Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z);
   %dir vector
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.x,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.y,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z);
   % transormed coordinates
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.UBJ_t_coord = convert_coordinate_frame(Smi.Car.FRONT.LEFT.UBJ',Smi.Car.FRONT.LEFT.Dimension.plane_HUB.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_t_coord = convert_coordinate_frame(Smi.Car.FRONT.LEFT.LBJ',Smi.Car.FRONT.LEFT.Dimension.plane_HUB.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.TIE_t_coord = convert_coordinate_frame(Smi.Car.FRONT.LEFT.TIE_HUB',Smi.Car.FRONT.LEFT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.UBJ_LBJ     = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.UBJ_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_TIE     = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.TIE_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.TIE_angle   = acos(dot(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_TIE,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.UBJ_LBJ)/norm(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_TIE)*norm(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.UBJ_LBJ));

  % HUB
  
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.WC_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.LEFT.WHEEL_CENTER',Smi.Car.FRONT.LEFT.Dimension.plane_HUB.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_HUB     = Vettore_Congiungente_No_Norm(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.WC_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.LBJ_t_coord);
  % rotate WC to World frame
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.x           = Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z;
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z           = Vettore_Congiungente(Smi.Car.FRONT.LEFT.UBJ,Smi.Car.FRONT.LEFT.LBJ);
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.y           = cross(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.x);
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.A           = Vector3_to_Matrix(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.x,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.y,Smi.Car.FRONT.LEFT.Dimension.plane_HUB.z);
  Smi.Car.FRONT.LEFT.Dimension.plane_HUB.WC_original = inv(Smi.Car.FRONT.LEFT.Dimension.plane_HUB.A);
  
%% Elastic plane
 %z
  Smi.Car.FRONT.LEFT.Dimension.z_Dir_El        = Normal_Plane(Smi.Car.FRONT.LEFT.CRANK_CENTEREVOLUTE,Smi.Car.FRONT.LEFT.DAMPER_CENTEREVOLUTE,Smi.Car.FRONT.LEFT.UW_PUSHROD);
 %y
  Smi.Car.FRONT.LEFT.Dimension.y_Dir_El        = Vettore_Congiungente(Smi.Car.FRONT.LEFT.DAMPER_CENTEREVOLUTE,Smi.Car.FRONT.LEFT.CRANK_CENTEREVOLUTE);
 %x
  Smi.Car.FRONT.LEFT.Dimension.x_Dir_El        = cross(Smi.Car.FRONT.LEFT.Dimension.y_Dir_El , Smi.Car.FRONT.LEFT.Dimension.z_Dir_El);

 %rotaion matrix
  Smi.Car.FRONT.LEFT.Dimension.El_Plane        = Vector3_to_Matrix(Smi.Car.FRONT.LEFT.Dimension.x_Dir_El,Smi.Car.FRONT.LEFT.Dimension.y_Dir_El,Smi.Car.FRONT.LEFT.Dimension.z_Dir_El);

  
  
 %% UW PLANE

   %z
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.z           = Normal_Plane(Smi.Car.FRONT.LEFT.UBJ,Smi.Car.FRONT.LEFT.A,Smi.Car.FRONT.LEFT.B);
   %y
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.y           = Vettore_Congiungente(Smi.Car.FRONT.LEFT.UBJ,Smi.Car.FRONT.LEFT.A);
   %x
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.x           = cross(Smi.Car.FRONT.LEFT.Dimension.plane_UW.y , Smi.Car.FRONT.LEFT.Dimension.plane_UW.z);
   %dir vector
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.LEFT.Dimension.plane_UW.x,Smi.Car.FRONT.LEFT.Dimension.plane_UW.y,Smi.Car.FRONT.LEFT.Dimension.plane_UW.z);
   % transormed coordinates
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.LEFT.A',Smi.Car.FRONT.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.B_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.LEFT.B',Smi.Car.FRONT.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.LEFT.UBJ',Smi.Car.FRONT.LEFT.Dimension.plane_UW.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.PS_t_coord   = convert_coordinate_frame(Smi.Car.FRONT.LEFT.UW_PUSHROD',Smi.Car.FRONT.LEFT.Dimension.plane_UW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_UBJ        = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_B        = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_UW.B_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_UW.wish_angle   = acos(dot(Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_UBJ,Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_B)...
                                                            /norm(Smi.Car.FRONT.LEFT.Dimension.plane_UW.A_UBJ)*norm(Smi.Car.FRONT.LEFT.Dimension.plane_UW.UBJ_B));

%% LW PLANE

   %z
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.z           = Normal_Plane(Smi.Car.FRONT.LEFT.LBJ,Smi.Car.FRONT.LEFT.C,Smi.Car.FRONT.LEFT.D);
   %y
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.y           = Vettore_Congiungente(Smi.Car.FRONT.LEFT.LBJ,Smi.Car.FRONT.LEFT.C);
   %x
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.x           = cross(Smi.Car.FRONT.LEFT.Dimension.plane_LW.y , Smi.Car.FRONT.LEFT.Dimension.plane_LW.z);
   %dir vector
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.dir_vec     = Vector3_to_Matrix(Smi.Car.FRONT.LEFT.Dimension.plane_LW.x,Smi.Car.FRONT.LEFT.Dimension.plane_LW.y,Smi.Car.FRONT.LEFT.Dimension.plane_LW.z);
   % transormed coordinates
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.C_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.LEFT.C',Smi.Car.FRONT.LEFT.Dimension.plane_LW.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.D_t_coord    = convert_coordinate_frame(Smi.Car.FRONT.LEFT.D',Smi.Car.FRONT.LEFT.Dimension.plane_LW.dir_vec);
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_t_coord  = convert_coordinate_frame(Smi.Car.FRONT.LEFT.LBJ',Smi.Car.FRONT.LEFT.Dimension.plane_LW.dir_vec);
   % UBJ -> LBJ
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.C_LBJ        = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_LW.C_t_coord);
   % LBJ -> TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_D        = Vettore_Congiungente(Smi.Car.FRONT.LEFT.Dimension.plane_LW.D_t_coord,Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_t_coord);
   % angle LBJ-UBJ and LBJ-TIE
  Smi.Car.FRONT.LEFT.Dimension.plane_LW.wish_angle   = acos(dot(Smi.Car.FRONT.LEFT.Dimension.plane_LW.C_LBJ,Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_D)...
                                                            /norm(Smi.Car.FRONT.LEFT.Dimension.plane_LW.C_LBJ)*norm(Smi.Car.FRONT.LEFT.Dimension.plane_LW.LBJ_D));


%% extrusion profiles

% Crank extrusion profile   

  Crank_Extrusion_Matrix_FL;
  
% Tyre extrusion profile   

  Smi.Car.FRONT.LEFT.Dimension.tyre_profile     = Tyre_extrusion(Smi.Car.FRONT.LEFT.TYRE_DIAMETER,Smi.Car.FRONT.LEFT.TYRE_WIDTH);
  
% OUTER ACTUATOR extrusion profile   

  Smi.Car.FRONT.LEFT.Dimension.OUT_ACT_profile  = OUT_ACT_extrusion(Smi.Graphic.OUT_ACT_diam,Smi.Graphic.OUT_ACT_thick);
  
% ACTUATOR extrusion profile   

  Smi.Car.FRONT.LEFT.Dimension.ACT_profile      = ACT_extrusion(Smi.Graphic.ACT_hole,Smi.Graphic.ACT_height,Smi.Graphic.ACT_diam);
  
% initial rack displacement

  Smi.Car.FRONT.LEFT.Dimension.Rack_initial     = Smi.Car.FRONT.LEFT.initial_steering*Smi.Car.FRONT.LEFT.rack_ratio;



