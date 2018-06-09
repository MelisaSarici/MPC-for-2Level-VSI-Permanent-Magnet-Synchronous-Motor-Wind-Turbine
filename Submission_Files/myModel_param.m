% RUN THIS SCRIPT FIRST TO LOAD REQUIRED DATA TO SIMULATION

Lambdad=1e-5; 
Lambdaq=1e-5; 

% alpbet=[2/3, -1/3, -1/3; 0, sqrt(3)/3, -sqrt(3)/3 ];
Vdc=540;
ma=0.8;
wg=61.25;
Ts=1e-3;
end_time=2; %run for tend secs
%P_ref=35e3;
% Here, the synchronous machine parameters are defined:

ri=0.1;  % Grid side converter harmonic filter resistance
Li= 0.16e-3; % Grid side converter harmonic filter inductance

s_1=[0;0;0];
s_2=[1;0;0];
s_3=[1;1;0];
s_4=[0;1;0];
s_5=[0;1;1];
s_6=[0;0;1];
s_7=[1;0;1];
s_8=[1;1;1];

T_a=[1 -0.5 -0.5;0 sqrt(3)/2 -sqrt(3)/2]*2/3;
