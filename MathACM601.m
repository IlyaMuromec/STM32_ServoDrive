
%% ?????????????
close all
clear
clc

% ????????? ACM601 -------------------------------------------------
R = 0.75; % ???? ???????
Ls = 2e-3;  % ????????????? ???????
Te = R*Ls;
zp = 4; % ?????????? ??? ???????
ce = 0.0712; % ?/(???/?)- ??????
cm = 0.0712; % N/A
J = 0.11e-4;

% ????????????? ???????----------------------------------------------------
tau=0.00; % 0.02 ??????? ?????
delU=0; % ?????????? ???????? ??????
k=2; % ??? ????????? - sqrt(3), ??? ?????????????? - 2 
Kv=1; % ???????? ????????? ???????-???????????? ???????, ????/?
Z=20; % ????????? ??????
Tpwm=0.5e-4;
Ts1=0.5e-4; % ?????? ???
Ts2=0.5e-3; % 
T0=1e-6; % ?????? ?????????????

PWM_MAX=Tpwm*160000000/2;
% ?????-?????????????:
add_error=0; % ???/???? ?????? ????????? ????
turn_M=0; % ???/???? ???????????? ???????????
Sw=0; % ???????????? ??????? ?? ?????? ?? ???????-???????????? ??????
comp=0; % ??????????? ????????? ???????
PreMod_3=0; % ????????? ????????????? 3-? ?????????. ?? ?????? ??? k! 

% ????????? ?????????------------------------------------------------------
Udc=8; % ???????
Cf = 100e-6;
Rshunt=0.33;
Ron=0.73*0.001;
Kshunt=0.001;
Vf=0.7*0.001;
Isc=Udc/R;
Kenc=2000/(2*pi);
Kopa=1.527;
Ubias=1.558;
KADC=(2^12-1)/3.3;
kdt=Rshunt*Kopa*KADC; % ??????????? ??????? ????
Ibias=Ubias/Rshunt/Kopa;
Kob=1/R*(Udc-2*delU)/k;
kda=1; kdb=0.99; kdc=1.015; % ????????????????? ??????
dia=0.002*Isc*kdt; dib=0.003*Isc*kdt; dic=-0.009*Isc*kdt; % ?????????? ??????
Ra=R; Rb=R; Rc=R; Lsa=Ls; Lsb=Ls; Lsc=Ls; % ???????? ???????????

% ???????? ??????
Nc=72;
Ns=36;
Q1=-90.7*pi/180;
Q2=-61*pi/180;
Mm1=2.232;
Mm2=0.565;
Msc1=2.232/Ns;
Msc2=0.565/Ns;
%alfa=[0:2*pi/(100*Nc):4*pi/Nc];
%Mz=Mm1*sin(Nc*alfa+Q1)+Mm2*sin(2*Nc*alfa+Q2);
%plot(alfa,Mz)
%grid on

%--------------------------------------------------------------------------
% ??????? ????????????? ????????
% ?????? ????
% ?? ?????????????? ??????????????? ?????: Kob/(Te*s+1)
% ?? ???????????? ??: 1/Tt*s-> ?? ??????????: (Kp*s+Ki)/s=Ki(Kp/Ki s+1)/s
% Kp/Ki=Te; 1/(Ki*Kob)=Tt -> Ki=1/Kob*Tt ->Kp=Te*Ki
% Ki1=1/(Kob*(Tt+dTs/2)); Kp1=(Te/Kob*(Tt+dTs/2));
%--------------------------------------------------------------------------
Tz=Ts1/2;
Tt=Ts1*10; % ???????? ??????????????
Ki1=1/(Kob*Tt);
Kp1=Te*Ki1
Ki1d=Ki1*Ts1
% ?????? ???????? - ??
% ?? ???????????? ??????? ????????: 1/(Tt*s+1)*3/2*ce*1/(J*s)
% ?? ???????????? ??: (4*Tmu*s+1)/(8*(Tmu*s)^2(Tmu*s+1)) || 1/(2Tmu(Tmu*s+1))
% ?? ??????????: (Kp*s+Ki)/s=Ki(Kp/Ki s+1)/s || Kp
% Ki2*3/2*ce*1/J = 1/(8*Tmu^2) || Kp*3/2*ce/J = 1/(2Tmu)
% 4*Tmu = Kp/Ki || Kp=2Tmu*2/3*J/ce
%--------------------------------------------------------------------------
Tmu=10*Tt;
Ki2=J*2/(3*8*Tmu^2*ce);
Kp2=4*Tmu*Ki2
Ki2d=Ki2*Ts2
%Ki2=0;
%Kp2=2*J/(3*ce*2*Tmu);