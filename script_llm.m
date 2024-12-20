close all
clear
clc

dt=0.01;
tmax=1000.0;

coupling = 'a' ; % working fine
% coupling = 'v' ; % not working (simulink model unstable)

m1=10;
k1=100;

m2=20;
k2=1200;

M1=m1*eye(2);
K1=k1*[2,-1;-1,1];
C1=M1*2.0+K1*0.02;


M2=m2*eye(2);
K2=k2*[2,-1;-1,1];
C2=M2*2.0+K2*0.02;

I=eye(2);
O=zeros(2);

A1=[O,I;-M1\K1,-M1\C1];
B1=[0;0;M1\[0;1]];

if strcmp(coupling,'v')
    G1=[0,1,0,0];
elseif strcmp(coupling,'a') %#ok<*UNRCH>
    G1=[0,0,0,1];
end

H1=[0;0;0;1];
B1b=-1;
G1b=-1;

A2=[O,I;-M2\K2,-M2\C2];
B2=[0;0;M2\[0;1]];
if strcmp(coupling,'v')
    G2=[0,1,0,0];
elseif strcmp(coupling,'a') %#ok<*UNRCH>
    G2=[0,0,0,1];
end
H2=[0;0;0;0];
B2b=-1;
G2b=-1;