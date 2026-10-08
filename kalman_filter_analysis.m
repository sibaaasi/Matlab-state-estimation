% Kalman filtering and steady-state estimation-error comparisons.
% Requires MATLAB with Control System Toolbox.
% Q and R are white-noise intensities; this script computes steady-state
% estimators and covariances rather than simulating noisy trajectories.

clear;
clc;
close all;


%% Question 1C + 1D


Aa = [-5 -6 1;
       1  0 0;
       0  0 -1];

Ba = [0;
      0;
      1];

Ca = [3 1 0];

Da = 0;

Q = 1;
R = 1;

sys1 = ss(Aa,Ba,Ca,Da);

[kest1,L1,P1] = kalman(sys1,Q,R);

disp('Question 1C')
disp('Kalman gain L =')
disp(L1)

disp('Estimation error covariance matrix P =')
disp(P1)

disp('Kalman estimator poles =')
disp(eig(kest1.A))

J_q1 = trace(P1);

disp('Question 1D: Error energy J =')
disp(J_q1)


%% Question 2E


A2 = [0 1;
      1 -2];

B1_2 = [0;
        1];

C2 = [0 1];

%% First gain: Ko1 = [2;2]
Ko1 = [2;
       2];

Ae1 = A2 - Ko1*C2;
Be1 = [B1_2 -Ko1];

poles1 = eig(Ae1);

P_ko1 = lyap(Ae1, Be1*Be1');

J_ko1 = trace(P_ko1);


disp('Question 2E - First gain Ko1 = [2;2]')
disp('Observer poles =')
disp(poles1)

disp('Covariance matrix P1 =')
disp(P_ko1)

disp('Error energy J1 =')
disp(J_ko1)


%% Second gain: Ko2 = [3;1]
Ko2 = [3;
       1];

Ae2 = A2 - Ko2*C2;
Be2 = [B1_2 -Ko2];

poles2 = eig(Ae2);

P_ko2 = lyap(Ae2, Be2*Be2');

J_ko2 = trace(P_ko2);

disp('Question 2E - Second gain Ko2 = [3;1]')
disp('Observer poles =')
disp(poles2)

disp('Covariance matrix P2 =')
disp(P_ko2)

disp('Error energy J2 =')
disp(J_ko2)


%% Question 2H

D2 = 0;

sys2 = ss(A2,B1_2,C2,D2);

[kest2,L2,P2] = kalman(sys2,Q,R);

Jmin = trace(P2);

disp('Question 2H')
disp('Optimal Kalman gain L =')
disp(L2)

disp('Covariance matrix P =')
disp(P2)

disp('Estimator poles =')
disp(eig(kest2.A))

disp('Minimum estimation error energy Jmin =')
disp(Jmin)


%% Question 3B

A3 = [-5 1;
      -6 0];

C3 = [1 0];
D3 = 0;

%% Minimum-phase system
Bmp = [1;
       6];

sysMP = ss(A3,Bmp,C3,D3);

[kestMP,Lmp,Pmp] = kalman(sysMP,Q,R);

Jmp = trace(Pmp);

disp('Question 3B - Minimum Phase')
disp('Kalman gain Lmp =')
disp(Lmp)

disp('Covariance matrix Pmp =')
disp(Pmp)

disp('Estimator poles =')
disp(eig(kestMP.A))

disp('Error energy Jmp =')
disp(Jmp)


%% Non-minimum-phase system
Bnmp = [1;
        -6];

sysNMP = ss(A3,Bnmp,C3,D3);

[kestNMP,Lnmp,Pnmp] = kalman(sysNMP,Q,R);

Jnmp = trace(Pnmp);

disp('Question 3B - Non-Minimum Phase')
disp('Kalman gain Lnmp =')
disp(Lnmp)

disp('Covariance matrix Pnmp =')
disp(Pnmp)

disp('Estimator poles =')
disp(eig(kestNMP.A))

disp('Error energy Jnmp =')
disp(Jnmp)
