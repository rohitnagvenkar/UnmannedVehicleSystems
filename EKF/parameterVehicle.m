%%%%%%%%%%%%%%% Track Vehicle Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
b = 0.305;%0.3075;%0.3295;%0.30520258 ;%0.30.305202580520258;%0.29933864;%0.20;%;%%%0.29933864 ; % [m] Effective Platform Width = Diagonal length
             % Actual Width = 0.3556 m
rNominal = 0.06;%0.052959;%0.06; % [m] Nominal Wheel Radius

Vmax = 1.5; %132/866.1417; % [m/s] Maximum speed of the vehicle
% wMax = Vmax/rNominal; % [rad/s] Maximum angular speed of wheels % NOT USED


rr = 1*rNominal; %% Effective vehicle right wheel 
rl = 1*rNominal; %%  1%% Effective vehicle left wheel to represent inaccuracy in the vehicle

%%%%%%%%%%%%%%% Encoder Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
eTick = 1050;%1070.57;%1070;%;%1070.57;%236.8852;%900; % 866.1417-905.5118; % [ticks/m] number of ticks per 1 m of vehicle translation % from 22-23 [ticks/inch]
TsampleEncoder = 1/10; %1/100; % 0.1 [s] Encoder sample time
%%
%%%%%%%%%%%%%%% Duty Cycle -> Speed - Conversion %%%%%%%%%%%%%%%%%%%%%%%%%
uv = -100;
vs = -uv;
dcArray = [uv  -30 0  30  vs];
wArray  = [-Vmax, 0, 0,  0, Vmax]./rNominal;

%%%%%%%%%%%%%%%%%% Initial Conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%
xIC = 0;
yIC = 0;
thetaIC = 0*(pi/180);
baseSpeed=1.25;