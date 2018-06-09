%% Model Parameters
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 
clear all;
%close all;
%%
Vdc=540;
ma=0.8;
wg=61.25;
Ts=1e-4;
tend=0.003;
% Here, the synchronous machine parameters are defined:

ri=0.1;  % Grid side converter harmonic filter resistance
Li= 0.16e-3; % Grid side converter harmonic filter inductance

A11=1-(ri*Ts)/Li;
A12=wg*Ts;
A21=-A12;
A22=A11;

B11=Ts/Li;
B22=B11;
B12=0;
B21=0;

A=[A11 A12; A21 A22];
B=[B11 B12; B21 B22];
v_dg=100;
v_qg=100;
C=1.5*[v_dg v_qg];
D=0;
x0 = [350; 350];
% Optimal control solution for $N = 4$
G = [zeros(2,2) zeros(2,2) zeros(2,2) zeros(2,2); B zeros(2,2) zeros(2,2) zeros(2,2); ...
    A*B B zeros(2,2) zeros(2,2); A^2*B A*B B zeros(2,2); A^3*B A^2*B A*B B];
H = [eye(2); A; A^2; A^3; A^4];
R = 1e4*eye(2);
%Q=C'*C;
Q = eye(2); %states vs input
Pinf = dare(A,B,Q,R,zeros(2,2),eye(2) );
Kinf = inv(R+B'*Pinf*B)*B'*Pinf*A;
P = dlyap( (A-B*Kinf)',Q+Kinf'*R*Kinf); % this is lyapunaov eq. for dual mode 
Qf = P; % this is the final state after N steps for dual mode
Qbar = blkdiag(Q,Q,Q,Q,Qf); % then with the Q and Qf at hand calculated for step k
Rbar = blkdiag(R,R,R,R);
M = G'*Qbar*G + Rbar;
umin = -360;
umax =360;
lb = [umin;umin;umin;umin];
ub = [umax;umax;umax;umax];
% Apply MPC steps
xVec(:,1) = x0;
yVec(1) = C*x0;
uVec = [];
for kk = 1:1:tend/Ts
    alpha = G'*Qbar*H*xVec(:,kk);
    Usol = quadprog(M,alpha',[],[],[],[],lb,ub);%% MPC here with every changing state alpha
    % changes and this makes the next state's quadratic cost function with N horizon
    uVec(:,kk) = Usol(1:2);
    xVec(:,kk+1) = A*xVec(:,kk) + B*uVec(:,kk);
    yVec(kk+1) = C*xVec(:,kk+1);
end
            



% %% 
uVec = [uVec uVec(:,end)];
tVec = [0:Ts:tend];




subplot(2,2,1)
stairs(tVec,uVec(1,:),'LineWidth',2);
hold all;
stairs(tVec,uVec(2,:),'--','LineWidth',2);

xlabel('time [sec]')
grid
ylabel('$u_1$')
title('Inputs  $V_d$ ')
legend('$V_d$', '$V_q$')

subplot(2,2,2)
xlabel('time [sec]')
ylabel('$y$')
stairs(tVec,yVec,'LineWidth',2);
grid
hold all;
title('Approximated Power ')

% stairs(tVec,uVec(2,:),'LineWidth',2);
% hold all;
% 
% title('Inputs  $V_q$')
% grid
% xlabel('time [sec]')
% ylabel('$u_2$')
subplot(2,2,3)
stairs(tVec,[1 0]*xVec,'LineWidth',2)
hold all;
grid
xlabel('time [sec]')
ylabel('$i_d$')
title('State $i_d$')
subplot(2,2,4)
stairs(tVec,[0 1]*xVec,'LineWidth',2)
hold all;
grid
xlabel('time [sec]')
ylabel('$i_q$')
title('State $i_q$')
