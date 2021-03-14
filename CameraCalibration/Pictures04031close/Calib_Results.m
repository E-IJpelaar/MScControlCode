% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1145.701654192264414 ; 1143.471697077219233 ];

%-- Principal point:
cc = [ 672.179072583248512 ; 540.658251213013614 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.013845332995299 ; 0.022968470430649 ; -0.003452965355122 ; -0.002134864201992 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 28.644974800355779 ; 26.376369140722272 ];

%-- Principal point uncertainty:
cc_error = [ 38.422975955295087 ; 28.511302146696330 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.037295794084076 ; 0.069969685627125 ; 0.007470177694101 ; 0.010631458920971 ; 0.000000000000000 ];

%-- Image size:
nx = 1359;
ny = 895;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 23;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.192555e+00 ; 2.057034e+00 ; 1.731708e-01 ];
Tc_1  = [ 3.264239e+00 ; -5.184627e+01 ; 2.229551e+02 ];
omc_error_1 = [ 2.835375e-02 ; 2.288662e-02 ; 5.113494e-02 ];
Tc_error_1  = [ 7.583776e+00 ; 5.530783e+00 ; 5.778086e+00 ];

%-- Image #2:
omc_2 = [ 2.109041e+00 ; 2.147653e+00 ; -2.762616e-02 ];
Tc_2  = [ 3.232071e+01 ; -5.069732e+01 ; 2.706192e+02 ];
omc_error_2 = [ 2.898681e-02 ; 2.560545e-02 ; 5.600787e-02 ];
Tc_error_2  = [ 9.190126e+00 ; 6.755609e+00 ; 7.016882e+00 ];

%-- Image #3:
omc_3 = [ 1.962870e+00 ; 1.968633e+00 ; 2.232081e-01 ];
Tc_3  = [ 5.902559e+01 ; -5.519235e+01 ; 2.742209e+02 ];
omc_error_3 = [ 2.877915e-02 ; 2.508477e-02 ; 4.621649e-02 ];
Tc_error_3  = [ 9.413066e+00 ; 6.920427e+00 ; 7.609117e+00 ];

%-- Image #4:
omc_4 = [ -1.793731e+00 ; -1.694378e+00 ; 9.192815e-01 ];
Tc_4  = [ 7.925256e+01 ; -4.245074e+01 ; 3.775366e+02 ];
omc_error_4 = [ 3.107770e-02 ; 2.682910e-02 ; 4.461927e-02 ];
Tc_error_4  = [ 1.283801e+01 ; 9.495220e+00 ; 8.453457e+00 ];

%-- Image #5:
omc_5 = [ -1.850459e+00 ; -1.829492e+00 ; -3.718431e-01 ];
Tc_5  = [ 3.790191e+01 ; -1.244603e+02 ; 3.176202e+02 ];
omc_error_5 = [ 2.902270e-02 ; 3.747265e-02 ; 5.814212e-02 ];
Tc_error_5  = [ 1.110835e+01 ; 8.162216e+00 ; 1.002860e+01 ];

%-- Image #6:
omc_6 = [ -1.910443e+00 ; -1.903516e+00 ; -4.136348e-01 ];
Tc_6  = [ -2.143293e+01 ; -1.249520e+02 ; 3.077149e+02 ];
omc_error_6 = [ 3.378778e-02 ; 3.606518e-02 ; 6.535742e-02 ];
Tc_error_6  = [ 1.079583e+01 ; 7.925591e+00 ; 9.660277e+00 ];

%-- Image #7:
omc_7 = [ -1.867286e+00 ; -1.876573e+00 ; -2.893779e-01 ];
Tc_7  = [ -1.015697e+02 ; -1.173926e+02 ; 2.901257e+02 ];
omc_error_7 = [ 2.921264e-02 ; 2.524827e-02 ; 6.341466e-02 ];
Tc_error_7  = [ 1.019814e+01 ; 7.460068e+00 ; 1.024912e+01 ];

%-- Image #8:
omc_8 = [ 1.763798e+00 ; 1.708838e+00 ; 7.238017e-01 ];
Tc_8  = [ -1.775619e+02 ; -7.587656e+01 ; 3.815072e+02 ];
omc_error_8 = [ 2.794081e-02 ; 3.187421e-02 ; 4.900999e-02 ];
Tc_error_8  = [ 1.436625e+01 ; 1.005347e+01 ; 1.300843e+01 ];

%-- Image #9:
omc_9 = [ 1.754901e+00 ; 1.606683e+00 ; 7.328589e-01 ];
Tc_9  = [ -1.646870e+02 ; -1.308454e+02 ; 3.180411e+02 ];
omc_error_9 = [ 2.323124e-02 ; 3.080141e-02 ; 4.050257e-02 ];
Tc_error_9  = [ 1.231501e+01 ; 8.489230e+00 ; 1.235278e+01 ];

%-- Image #10:
omc_10 = [ 1.689270e+00 ; 1.628329e+00 ; 6.368612e-01 ];
Tc_10  = [ -1.140872e+02 ; -6.626801e+00 ; 2.960654e+02 ];
omc_error_10 = [ 2.592201e-02 ; 2.377721e-02 ; 3.986320e-02 ];
Tc_error_10  = [ 1.069860e+01 ; 7.679319e+00 ; 9.320702e+00 ];

%-- Image #11:
omc_11 = [ 2.097773e+00 ; 1.833607e+00 ; -1.961881e-01 ];
Tc_11  = [ -2.143118e+01 ; -2.798745e+01 ; 2.509184e+02 ];
omc_error_11 = [ 2.452670e-02 ; 2.352359e-02 ; 4.629818e-02 ];
Tc_error_11  = [ 8.436113e+00 ; 6.190726e+00 ; 6.221428e+00 ];

%-- Image #12:
omc_12 = [ 2.206797e+00 ; 2.009299e+00 ; 5.671039e-02 ];
Tc_12  = [ -1.737713e+01 ; -2.395800e+01 ; 3.688196e+02 ];
omc_error_12 = [ 3.970691e-02 ; 3.347871e-02 ; 7.804824e-02 ];
Tc_error_12  = [ 1.239751e+01 ; 9.182223e+00 ; 1.027146e+01 ];

%-- Image #13:
omc_13 = [ 2.238557e+00 ; 2.027401e+00 ; 1.904764e-01 ];
Tc_13  = [ -7.034396e+00 ; -1.842813e+01 ; 4.714828e+02 ];
omc_error_13 = [ 4.927786e-02 ; 3.983494e-02 ; 9.574582e-02 ];
Tc_error_13  = [ 1.583613e+01 ; 1.175660e+01 ; 1.428908e+01 ];

%-- Image #14:
omc_14 = [ 2.138437e+00 ; 2.017191e+00 ; 2.727244e-02 ];
Tc_14  = [ -1.392979e+00 ; -5.506870e+01 ; 2.077808e+02 ];
omc_error_14 = [ 2.583375e-02 ; 2.337460e-02 ; 4.764670e-02 ];
Tc_error_14  = [ 7.084567e+00 ; 5.139751e+00 ; 5.310780e+00 ];

%-- Image #15:
omc_15 = [ 1.941588e+00 ; 2.039466e+00 ; -1.047094e+00 ];
Tc_15  = [ 1.358234e+01 ; -1.391206e+01 ; 2.991158e+02 ];
omc_error_15 = [ 2.073346e-02 ; 2.641183e-02 ; 4.686430e-02 ];
Tc_error_15  = [ 1.001817e+01 ; 7.391447e+00 ; 6.390451e+00 ];

%-- Image #16:
omc_16 = [ -2.118285e+00 ; -2.002468e+00 ; 4.598231e-01 ];
Tc_16  = [ -1.119549e+01 ; -3.323283e+01 ; 2.343054e+02 ];
omc_error_16 = [ 2.499984e-02 ; 2.450518e-02 ; 4.873129e-02 ];
Tc_error_16  = [ 7.831144e+00 ; 5.766060e+00 ; 5.630928e+00 ];

%-- Image #17:
omc_17 = [ 1.767648e+00 ; 1.688493e+00 ; 6.623793e-01 ];
Tc_17  = [ -6.373052e+01 ; -4.300228e+01 ; 1.613505e+02 ];
omc_error_17 = [ 2.422918e-02 ; 2.157256e-02 ; 3.537412e-02 ];
Tc_error_17  = [ 5.818519e+00 ; 4.148557e+00 ; 5.123211e+00 ];

%-- Image #18:
omc_18 = [ 1.805669e+00 ; 1.613530e+00 ; -2.385832e-01 ];
Tc_18  = [ -1.293049e+02 ; -1.024903e+01 ; 2.813320e+02 ];
omc_error_18 = [ 1.945980e-02 ; 2.986391e-02 ; 3.829710e-02 ];
Tc_error_18  = [ 9.639394e+00 ; 7.348861e+00 ; 9.078036e+00 ];

%-- Image #19:
omc_19 = [ 1.789600e+00 ; 1.689370e+00 ; 8.519396e-02 ];
Tc_19  = [ -1.103828e+02 ; -2.348592e+01 ; 2.576406e+02 ];
omc_error_19 = [ 2.140617e-02 ; 2.766289e-02 ; 3.781054e-02 ];
Tc_error_19  = [ 9.064031e+00 ; 6.666237e+00 ; 7.878419e+00 ];

%-- Image #20:
omc_20 = [ 1.797860e+00 ; 1.762494e+00 ; -5.795608e-01 ];
Tc_20  = [ 3.964694e+01 ; -3.105770e+00 ; 3.430951e+02 ];
omc_error_20 = [ 2.426105e-02 ; 2.576675e-02 ; 4.329564e-02 ];
Tc_error_20  = [ 1.152725e+01 ; 8.553032e+00 ; 8.663276e+00 ];

%-- Image #21:
omc_21 = [ 1.875179e+00 ; 1.768760e+00 ; 7.313556e-01 ];
Tc_21  = [ -1.424417e+02 ; -1.005799e+02 ; 2.998958e+02 ];
omc_error_21 = [ 2.548332e-02 ; 3.023302e-02 ; 4.683473e-02 ];
Tc_error_21  = [ 1.140213e+01 ; 7.939446e+00 ; 1.055505e+01 ];

%-- Image #22:
omc_22 = [ 1.970930e+00 ; 1.932536e+00 ; 4.320911e-01 ];
Tc_22  = [ 2.340720e+01 ; -6.042938e+01 ; 2.512683e+02 ];
omc_error_22 = [ 2.892211e-02 ; 2.454675e-02 ; 4.423229e-02 ];
Tc_error_22  = [ 8.555396e+00 ; 6.267748e+00 ; 6.966670e+00 ];

%-- Image #23:
omc_23 = [ -1.966518e+00 ; -1.929783e+00 ; 7.243948e-01 ];
Tc_23  = [ -7.398983e+01 ; -5.688238e+01 ; 3.250098e+02 ];
omc_error_23 = [ 3.245957e-02 ; 2.350248e-02 ; 4.789269e-02 ];
Tc_error_23  = [ 1.111375e+01 ; 8.192533e+00 ; 7.673884e+00 ];

