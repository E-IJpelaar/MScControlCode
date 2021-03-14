% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1407.577913392281062 ; 1398.792505795457373 ];

%-- Principal point:
cc = [ 596.030917383023848 ; 585.435986772097863 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.108428335174581 ; -0.381361287923466 ; 0.011887673733487 ; -0.029143503973231 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1160.160820732580760 ; 1129.197432721677842 ];

%-- Principal point uncertainty:
cc_error = [ 276.534219279043953 ; 68.634120445573387 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.243288583565309 ; 1.358474076808418 ; 0.010898694871009 ; 0.035108493502753 ; 0.000000000000000 ];

%-- Image size:
nx = 1359;
ny = 895;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 6;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.134155e+00 ; -2.089845e+00 ; 2.431067e-01 ];
Tc_1  = [ -1.814357e+01 ; -5.886141e+01 ; 2.642659e+02 ];
omc_error_1 = [ 8.299117e-02 ; 7.939115e-02 ; 2.181988e-01 ];
Tc_error_1  = [ 5.194031e+01 ; 1.285247e+01 ; 2.120385e+02 ];

%-- Image #2:
omc_2 = [ -2.135310e+00 ; -2.089858e+00 ; 2.421061e-01 ];
Tc_2  = [ -1.815890e+01 ; -5.881183e+01 ; 2.641015e+02 ];
omc_error_2 = [ 8.272887e-02 ; 7.907075e-02 ; 2.177728e-01 ];
Tc_error_2  = [ 5.190396e+01 ; 1.284478e+01 ; 2.119302e+02 ];

%-- Image #3:
omc_3 = [ -2.202073e+00 ; -2.169971e+00 ; 4.151578e-02 ];
Tc_3  = [ -1.724060e+01 ; -5.940741e+01 ; 2.335438e+02 ];
omc_error_3 = [ 2.980563e-02 ; 2.993149e-02 ; 8.512246e-02 ];
Tc_error_3  = [ 4.582418e+01 ; 1.150248e+01 ; 1.877158e+02 ];

%-- Image #4:
omc_4 = [ -2.206808e+00 ; -2.176681e+00 ; -3.354511e-03 ];
Tc_4  = [ -1.726304e+01 ; -5.950789e+01 ; 2.304898e+02 ];
omc_error_4 = [ 2.745444e-02 ; 2.842845e-02 ; 7.339545e-02 ];
Tc_error_4  = [ 4.539177e+01 ; 1.140550e+01 ; 1.850841e+02 ];

%-- Image #5:
omc_5 = [ -2.132867e+00 ; -2.085235e+00 ; 2.768138e-01 ];
Tc_5  = [ -1.822073e+01 ; -5.840618e+01 ; 2.640593e+02 ];
omc_error_5 = [ 8.527111e-02 ; 7.972492e-02 ; 2.324832e-01 ];
Tc_error_5  = [ 5.197115e+01 ; 1.284173e+01 ; 2.111288e+02 ];

%-- Image #6:
omc_6 = [ -2.135832e+00 ; -2.085137e+00 ; 2.664104e-01 ];
Tc_6  = [ -1.833134e+01 ; -5.841949e+01 ; 2.638454e+02 ];
omc_error_6 = [ 8.441849e-02 ; 7.925042e-02 ; 2.284873e-01 ];
Tc_error_6  = [ 5.190446e+01 ; 1.283004e+01 ; 2.112109e+02 ];

