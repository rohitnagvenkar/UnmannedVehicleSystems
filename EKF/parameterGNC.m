% Navigation Parameters
% if we want the estimation parameters identical to actual parameters
VmaxDR = Vmax; 
rNominalDR = rNominal;
bDR = 1*b;
eTickDR = eTick;
TsampleEncoderDR = TsampleEncoder;
TauEncoderDR = 0.1;
%%

%%%%%%%%%%%%%%%%%%% control parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1:speed, 2:angle
KP1=12; %20;    % 1 is velocity controller
KP2=70;%10;  %7 ->   % 2 is angle controller default 3
KI1= 0.0001;%10;%0.3; %0.001; %0.001;%10;
KI2= 0.0001; %0.001; %0.001;%1; % 3 -> default 0.001
KD1=0; %3; %0; %3; %0;%3;
KD2=5.7;% 3;%  %.5; %0; %0.5; %0;%0.5; %0.5 -> default 0
Tsample = 1/10; %1/100; %1/10;   %sampling rate -> 0.01
Tmodel=Tsample;
Tau1 = 0.01;     %time constant of filter 1
Tau2 = 0.1;%0.1;       %time constant of filter 2

% wheel speed - DC conversion
uv = -100;
vs = -uv;
wArrayC  = [-VmaxDR, -VmaxDR/100, 0,  VmaxDR/100, VmaxDR]./rNominalDR;
dcArrayC = [uv  -30 0  30  vs];

%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
rp1 =  0.7; % 1 -> [m] proximity circle to start slowing down
rp2 =  0.3;  % [m] radius of wayPoint proximity circle to switch to the next wayPoint
Vcom = (3/4)*VmaxDR; % [m/s] used when constant speed is commanded
a = 1;
b = 1;
c = 0.2;
d = 0.2;
%%%%%%%%%%%%%%%%%% Way Points %%%%%%%%%%%%%%%%%%%%%%%%%%%
X_array  = [ 1 1 0 0];%  0  0];%[0 -4]; 
Y_array  = [ 0 1 1 0];% -3  0];%[0 0]; 
%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%

Etheta= 0;