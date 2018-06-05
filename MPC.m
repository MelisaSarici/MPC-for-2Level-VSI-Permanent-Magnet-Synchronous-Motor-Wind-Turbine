% Some figure formatting
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finite Horizon MPC as quadratic program
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = [1.1 2; 0 0.95];
B = [0; 0.0787];
C = [-1 1];
D = 0;
x0 = [0.5;-0.5];
Ts = 1;
% Optimal control solution for $N = 4$
G = [zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1); B zeros(2,1) zeros(2,1) zeros(2,1); ...
    A*B B zeros(2,1) zeros(2,1); A^2*B A*B B zeros(2,1); A^3*B A^2*B A*B B];
H = [eye(2); A; A^2; A^3; A^4];
Q = C'*C;
R = .01;
% Q = eye(2);
Pinf = dare(A,B,Q,R,zeros(2,1),eye(2) );
Kinf = inv(R+B'*Pinf*B)*B'*Pinf*A;
% A*X*A' - X + Q = 0;  X = dlyap(A,Q)
P = dlyap( (A-B*Kinf)',Q+Kinf'*R*Kinf);
Qf = P;
Qbar = blkdiag(Q,Q,Q,Q,Qf);
Rbar = blkdiag(R,R,R,R);
M = G'*Qbar*G + Rbar;
% input bound: umin <= u <= umax
umin = -Inf;
umax = Inf;
umin = -4;
umax = 4;
lb = [umin;umin;umin;umin];
ub = [umax;umax;umax;umax];
% Apply MPC steps
xVec(:,1) = x0;
yVec(1) = C*x0;
uVec = [];
for kk = 1:30
    alpha = G'*Qbar*H*xVec(:,kk);
    Usol = quadprog(M,alpha',[],[],[],[],lb,ub);
    uVec(kk) = Usol(1);
    xVec(:,kk+1) = A*xVec(:,kk) + B*uVec(kk);
    yVec(kk+1) = C*xVec(:,kk+1);
%     Xsol(:,1) = xVec(:,kk);
%     Xsol(:,2) = A*Xsol(:,1) + B*Usol(1);
%     Xsol(:,3) = A*Xsol(:,2) + B*Usol(2);
%     Xsol(:,4) = A*Xsol(:,3) + B*Usol(3);
%     Ysol(1) = C*Xsol(:,1);
%     Ysol(2) = C*Xsol(:,2);
%     Ysol(3) = C*Xsol(:,3);
%     Ysol(4) = C*Xsol(:,4);
end


uVec = [uVec uVec(end)];
tVec = [0:1:30];
% figure;
subplot(2,2,1)
stairs(tVec,uVec,'LineWidth',2);
hold all;
xlabel('time [sec]')
grid
ylabel('$u$')
title('Input $u$')
subplot(2,2,2)
stairs(tVec,C*xVec,'LineWidth',2)
hold all;
grid
xlabel('time [sec]')
ylabel('$y$')
title('Output $y$')
subplot(2,2,3)
stairs(tVec,[1 0]*xVec,'LineWidth',2)
hold all;
grid
xlabel('time [sec]')
ylabel('$x_1$')
title('State $x_1$')
subplot(2,2,4)
stairs(tVec,[0 1]*xVec,'LineWidth',2)
hold all;
grid
xlabel('time [sec]')
ylabel('$x_2$')
title('State $x_2$')
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);
% legend('$u_{max} = 1.5$','$u_{max} = 2.5$','$u_{max} = 4$')