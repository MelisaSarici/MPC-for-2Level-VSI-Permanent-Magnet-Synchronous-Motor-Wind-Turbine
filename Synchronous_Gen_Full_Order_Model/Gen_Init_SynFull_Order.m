clc;
clear all;
%% Machine parameters in pu
xd=1.79;
xdd=0.169;
xddd=0.135;
Td0d=2;
Td0dd=2.3;
xq=1.71;
xqd=0.228;
xqdd=0.2;
Tq0d=3;
Tq0dd=3.3;
H=2.86;
D=0;
Ra=0.01;
wB=50*2*pi;
w0=50*2*pi;
w_ref=1.01;    %   Initial speed of the machine.
Tddd= 0.0259;
Tdd = 0.4;
Tqdd=0.0463;
Tqd = 0.1073;
W0init = 1;
%%  Initilization of machine 
Pg0=0.01;
Ug0bar=1.0;  % taken as reference
%pure resistive load is assumed
I0bar=Pg0/Ug0bar;

Uq0bar=Ug0bar+(Ra+1i*xq)*I0bar;
deltag0=angle(Uq0bar);
Uq0=abs(Uq0bar);

VQg0=real(Ug0bar);
VDg0=imag(Ug0bar);
iD0=imag(I0bar);
iQ0=real(I0bar);

Ug0bardq=Ug0bar*(cos(deltag0)-1i*sin(deltag0));
I0bardq=I0bar*(cos(deltag0)-1i*sin(deltag0));

uqg0=real(Ug0bardq);
udg0=imag(Ug0bardq);
iq0=real(I0bardq);
id0=imag(I0bardq);
RL=uqg0/iq0;

Ufd0=Uq0-(xd-xq)*id0;
sid0=xd*id0+Ufd0;
siq0=xq*iq0;
sif0=sid0 + (xdd/(xd-xdd))*Ufd0;
sih0=sid0;
sig0=siq0;
sik0=siq0;

Uqdd0=((xdd-xddd)./xdd).*sih0 + ((xd-xdd)./xd).*(xddd./xdd).*sif0;
Uddd0=-(((xqd-xqdd)./xqd).*sik0 + ((xq-xqd)./xq).*(xqdd./xqd).*sig0);

Te0=sid0*iq0-siq0*id0;
Tm=Te0;

xfl=(xd*xdd)/(xd-xdd);
ifd0= (sif0-sid0 )/xfl;
IFD0=ifd0*xd;
Zmat = [Ra xqdd; -(xddd) Ra];
Ymat=inv(Zmat);
SigmaInv =1/0.05;
Tswitch=input('Enter the time at which Genarator is synchronized: ')

% Reference : Power System Stability and Control by Prabha Kunder.

