function mf = fuzzyMFParams()
%FUZZYMFPARAMS Shared membership-function parameters for the fuzzy torque
%vectoring controller (fuzzyTorqueVectoring.m). Centralized here so that
%diagnostic/visualization scripts (e.g. vizMF.m) stay in sync with the
%values actually driving the controller instead of keeping their own
%drifted copies.

% Membership functions for psi_dot_err (yaw rate error)
mf.psiDotErrMFs   = [-0.5, -0.10, -0.04, 0, 0.04, 0.10, 0.5]; % Centers for NB to PB
mf.psiDotErrWidth = 0.028; % Width of each MF

% Membership functions for psi_ddot_err (yaw acceleration error)
mf.psiDDotErrMFs   = [-8.5, -5.8, -3.95, 0, 3.95, 5.8, 8.5]; % Centers for NB to PB
mf.psiDDotErrWidth = 0.73; % Width of each MF

end
