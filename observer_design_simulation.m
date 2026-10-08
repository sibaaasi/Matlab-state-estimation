% Observer design and state-estimate feedback control.
% Requires MATLAB with Control System Toolbox.
% The local function at the end requires MATLAB R2016b or newer.

clear;
clc;
close all;

%% Question 1C - Ackermann observer design (first system)
A = [1 2;
    3 4];

C = [2 0];

desired_poles = [-5 -5];

Ko = acker(A', C', desired_poles)'

disp('First observer poles:')
disp(eig(A - Ko*C))

%% Question 1C - Ackermann observer design (second system)
A = [5 6;
    7 8];

C = [0 2];

desired_poles = [-5 -6];

Ko = acker(A', C', desired_poles)'

disp('Second observer poles:')
disp(eig(A - Ko*C))

%% Question 3 - Plant model
A = [-4 3 3;
     -8 6 8;
      2 -1 -3];

B = [1 0;
     0 1;
     0 0];

C = [1 0 4;
     0 1 0];

%% Question 3A 
Co = ctrb(A,B);
Ob = obsv(A,C);

rank_Co = rank(Co)
rank_Ob = rank(Ob)

%% Question 3B 
controller_poles = [-4 -5 -6];
observer_poles = [-6 -8 -12];

Kc = place(A,B,controller_poles);
Ko = place(A',C',observer_poles)';

disp('Kc =')
disp(Kc)

disp('Ko =')
disp(Ko)

disp('Poles of A-B*Kc:')
disp(eig(A-B*Kc))

disp('Poles of A-Ko*C:')
disp(eig(A-Ko*C))

%% Question 3C 
x0 = [10; 12; 17];
xhat0 = [0; 0; 0];

z0 = [x0; xhat0];
tspan = [0 2];

[t,z] = ode45(@(t,z) observer_system(t,z,A,B,C,Kc,Ko), tspan, z0);


x = z(:,1:3);
xhat = z(:,4:6);

%% Calculate y, u, and estimation error
y = (C*x')';
u = -(Kc*xhat')';
e = x - xhat;

%% Plot x and xhat
figure;
plot(t,x,'LineWidth',1.5);
hold on;
plot(t,xhat,'--','LineWidth',1.5);
grid on;
xlabel('Time [sec]');
ylabel('States');
legend('x_1','x_2','x_3','xhat_1','xhat_2','xhat_3');
title('Actual States and Estimated States');

%% Plot estimation error
figure;
plot(t,e,'LineWidth',1.5);
grid on;
xlabel('Time [sec]');
ylabel('Estimation Error');
legend('e_1','e_2','e_3');
title('Estimation Error: e = x - xhat');

%% Plot output y
figure;
plot(t,y,'LineWidth',1.5);
grid on;
xlabel('Time [sec]');
ylabel('Output');
legend('y_1','y_2');
title('System Output');

%% Plot control input u
figure;
plot(t,u,'LineWidth',1.5);
grid on;
xlabel('Time [sec]');
ylabel('Control Input');
legend('u_1','u_2');
title('Control Inputs');


function dz = observer_system(~,z,A,B,C,Kc,Ko)

    x = z(1:3);
    xhat = z(4:6);

    y = C*x;

    u = -Kc*xhat;

    xdot = A*x + B*u;

    xhatdot = A*xhat + B*u + Ko*(y - C*xhat);

    dz = [xdot;
          xhatdot];
end
