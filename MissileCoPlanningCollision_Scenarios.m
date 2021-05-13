clc;
clear;
close all;
format long;
yalmip clear;
disp('%--------------------------------------------------------------%');
disp(' #=============================================================#');
disp(' |             Cooperative Path Planning Using Dual-Loop       |');
disp(' | (TRAjectory GENeration using Sequence Convex Optimization)  |');
disp(' |     (MATLAB R2020a + ecos  Full Edition by Huan Jiang)      |');
disp(' |     (Official site: http://www.askyspace.com/paper code)    |');
disp(' |  This is  SKY REIN @ Beijing Institute of Tech.(01/29/2021) |');
disp(' #=============================================================#');


%% %------------------Modeling and Constraint setting------------%
disp('Modeling and Constraint Setting ...');
% scenario-1
x00  =  0; y00  = 0; z00  = 2000;
xf0  = 10000; yf0  = 10000; zf0  = 2000;
V00   = 190.0; Vf0 = 190.0;
psi00 = 0*pi/180; psif0 = -40*pi/180;
phi00 = 40*pi/180; phif0 = 90*pi/180;

x01  = 400; y01  = 0; z01  = 2000;
xf1  = 10400; yf1  = 10000; zf1  = 2000;
V01   = 200.0; Vf1 = 200.0;
psi01 = 0*pi/180; psif1 = -40*pi/180;
phi01 = 40*pi/180; phif1 = 90*pi/180;


x02  =  800; y02  = 0; z02  = 2000;
xf2  = 10800; yf2  = 10000; zf2  = 2000;
V02   = 210.0; Vf2 = 210.0;
psi02 = 0*pi/180; psif2 = -40*pi/180;
phi02 = 40*pi/180; phif2 = 90*pi/180;


x03  =  1200; y03  = 0; z03  = 2000;
xf3  = 11200; yf3  = 10000; zf3  = 2000;
V03   = 220.0; Vf3 = 220.0;
psi03 = 0*pi/180; psif3 = -40*pi/180;
phi03 = 40*pi/180; phif3 = 90*pi/180;


% x00  =  0; y00  = 0; z00  = 2000;
% xf0  = 10000; yf0  = 8000; zf0  = 10;
% V00   = 190.0; Vf0 = 190.0;
% psi00 = 0*pi/180; psif0 = -40*pi/180;
% phi00 = 40*pi/180; phif0 = 90*pi/180;
% 
% x01  = 400; y01  = 0; z01  = 2000;
% xf1  = 10000; yf1  = 8800; zf1  = 10;
% V01   = 200.0; Vf1 = 200.0;
% psi01 = 0*pi/180; psif1 = -40*pi/180;
% phi01 = 40*pi/180; phif1 = 90*pi/180;
% 
% 
% x02  =  800; y02  = 0; z02  = 2000;
% xf2  = 10000; yf2  = 9200; zf2  = 10;
% V02   = 210.0; Vf2 = 210.0;
% psi02 = 0*pi/180; psif2 = -40*pi/180;
% phi02 = 40*pi/180; phif2 = 90*pi/180;
% 
% 
% x03  =  1200; y03  = 0; z03  = 2000;
% xf3  = 10000; yf3  = 8400; zf3  = 10;
% V03   = 220.0; Vf3 = 220.0;
% psi03 = 0*pi/180; psif3 = -40*pi/180;
% phi03 = 40*pi/180; phif3 = 90*pi/180;

X_oc = [2000;6000];
Y_oc = [4000;4000];
R_oc = [1000;2000];
R_ca = 200;
%====================[-SPECS-DJI-MAVIC2 S-Model]=========================%
Speed_max = 200;            % Max Speed (near sea level) 20
Vl_max    = 100;            % Max level Speed 10
Des_max   = 30;             % Max Descent Speed 3
As_max    = 30;             % Max Ascent Speed 5
Accel_max = 3*9.82;             % Max Acceleration 12
pullz_min = .05*Speed_max; % Approximate minimum z acceleration
Tilt_max  = 30;            % Max Tilt Angle
m0        = 300;
%====================[-SPECS-DJI-MAVIC2 S-Model]=========================%

Speed_max0  = V00;
Speed_max1  = V01;
Speed_max2  = V02;
Speed_max3  = V03;

flagobs = 1;
flagcol = 1;
method = 'Projection';% method = 'Linearization';
N   = 50;    % Discretization points
tau = 1/N;
g = 9.82;
Lnorm = norm([xf0-x00;yf0-y00]);
Vnorm = sqrt(g*Lnorm);
anorm = g;
Tnorm = Lnorm/Vnorm;

x00  = x00/Lnorm;
xf0  = xf0/Lnorm;
y00  = y00/Lnorm;
yf0  = yf0/Lnorm;
z00  = z00/Lnorm;
zf0  = zf0/Lnorm;
Vx_00 = V00*cos(psi00)*cos(phi00)/Vnorm;
Vy_00 = V00*cos(psi00)*sin(phi00)/Vnorm;
Vz_00 = V00*sin(psi00)/Vnorm;
Vx_f0 = Vf0*cos(psif0)*cos(phif0)/Vnorm;
Vy_f0 = Vf0*cos(psif0)*sin(phif0)/Vnorm;
Vz_f0 = Vf0*sin(psif0)/Vnorm;

x01  = x01/Lnorm;
xf1  = xf1/Lnorm;
y01  = y01/Lnorm;
yf1  = yf1/Lnorm;
z01  = z01/Lnorm;
zf1  = zf1/Lnorm;
Vx_01 = V01*cos(psi01)*cos(phi01)/Vnorm;
Vy_01 = V01*cos(psi01)*sin(phi01)/Vnorm;
Vz_01 = V01*sin(psi01)/Vnorm;
Vx_f1 = Vf1*cos(psif1)*cos(phif1)/Vnorm;
Vy_f1 = Vf1*cos(psif1)*sin(phif1)/Vnorm;
Vz_f1 = Vf1*sin(psif1)/Vnorm;

x02  = x02/Lnorm;
xf2  = xf2/Lnorm;
y02  = y02/Lnorm;
yf2  = yf2/Lnorm;
z02  = z02/Lnorm;
zf2  = zf2/Lnorm;
Vx_02 = V02*cos(psi02)*cos(phi02)/Vnorm;
Vy_02 = V02*cos(psi02)*sin(phi02)/Vnorm;
Vz_02 = V02*sin(psi02)/Vnorm;
Vx_f2 = Vf2*cos(psif2)*cos(phif2)/Vnorm;
Vy_f2 = Vf2*cos(psif2)*sin(phif2)/Vnorm;
Vz_f2 = Vf2*sin(psif2)/Vnorm;

x03  = x03/Lnorm;
xf3  = xf3/Lnorm;
y03  = y03/Lnorm;
yf3  = yf3/Lnorm;
z03  = z03/Lnorm;
zf3  = zf3/Lnorm;
Vx_03 = V03*cos(psi03)*cos(phi03)/Vnorm;
Vy_03 = V03*cos(psi03)*sin(phi03)/Vnorm;
Vz_03 = V03*sin(psi03)/Vnorm;
Vx_f3 = Vf3*cos(psif3)*cos(phif3)/Vnorm;
Vy_f3 = Vf3*cos(psif3)*sin(phif3)/Vnorm;
Vz_f3 = Vf3*sin(psif3)/Vnorm;

Speedmax0 = Speed_max0/Vnorm;
Speedmax1 = Speed_max1/Vnorm;
Speedmax2 = Speed_max2/Vnorm;
Speedmax3 = Speed_max3/Vnorm;

Vlmax   = Vl_max/Vnorm;
Vzmin   = -Des_max/Vnorm;
Vzmax   = As_max/Vnorm;
amax    = Accel_max/anorm;%12
uzmin   = pullz_min/anorm;
Tiltmax = Tilt_max*pi/180;
s = 1;
tol = 1.0e-6;

Xoc = X_oc/Lnorm;
Yoc = Y_oc/Lnorm;
Roc = R_oc/Lnorm;
Rca = R_ca/Lnorm;
Umatrix = [cos(-30*pi/180) -sin(-30*pi/180);sin(-30*pi/180) cos(-30*pi/180)];
Sigmatrix = [1.4 0;0 0.8];
Aoc{1,1} = Umatrix*Sigmatrix*[1 0;0 1];% define the ellipse obstacle
Umatrix = [cos(-30*pi/180) -sin(-30*pi/180);sin(-30*pi/180) cos(-30*pi/180)];
Sigmatrix = [1.27 0;0 0.488];
Aoc{2,1} =Umatrix*Sigmatrix*[1 0;0 1];

%%
S = 20;
S=S+1;
Lambda(1,:) = 0;
T1k(1) = sqrt((xf0-x00)^2+(yf0-y01)^2)/(0.5*(V00+Vf0))/(N*tau);%
% Xk0(1,:) = linspace(x00,xf0,N);
% Yk0(1,:) = linspace(y00,yf0,N);
Zk0(1,:) = linspace(z00,zf0,N);
Xk0(1,:) = interp1([1,fix(N/2),N],[x00,xf0,xf0],1:N,'linear','extrap');
Yk0(1,:) = interp1([1,fix(N/2),N],[y00,0,yf0],1:N,'linear','extrap');
vxk0(1,:) = linspace(Vx_00,Vx_f0,N-1);
vyk0(1,:) = linspace(Vy_00,Vy_f0,N-1);
vzk0(1,:) = linspace(Vz_00,Vz_f0,N-1);
uxk0(1,:) = linspace(1,1,N-2)*0.001;
uyk0(1,:) = linspace(1,1,N-2)*0.001;
uzk0(1,:) = linspace(1,1,N-2)*0.001;

% Xk1(1,:) = linspace(x01,xf1,N);
% Yk1(1,:) = linspace(y01,yf1,N);
Xk1(1,:) = interp1([1,fix(N/2),N],[x01,xf1,xf1],1:N,'linear','extrap');
Yk1(1,:) = interp1([1,fix(N/2),N],[y01,0,yf1],1:N,'linear','extrap');
Zk1(1,:) = linspace(z01,zf1,N);
vxk1(1,:) = linspace(Vx_01,Vx_f1,N-1);
vyk1(1,:) = linspace(Vy_01,Vy_f1,N-1);
vzk1(1,:) = linspace(Vz_01,Vz_f1,N-1);
uxk1(1,:) = linspace(1,1,N-2)*0.001;
uyk1(1,:) = linspace(1,1,N-2)*0.001;
uzk1(1,:) = linspace(1,1,N-2)*0.001;

% Xk2(1,:) = linspace(x02,xf2,N);
% Yk2(1,:) = linspace(y02,yf2,N);
Xk2(1,:) = interp1([1,fix(N/2),N],[x02,xf2,xf2],1:N,'linear','extrap');
Yk2(1,:) = interp1([1,fix(N/2),N],[y02,0,yf2],1:N,'linear','extrap');
Zk2(1,:) = linspace(z02,zf2,N);
vxk2(1,:) = linspace(Vx_02,Vx_f2,N-1);
vyk2(1,:) = linspace(Vy_02,Vy_f2,N-1);
vzk2(1,:) = linspace(Vz_02,Vz_f2,N-1);
uxk2(1,:) = linspace(1,1,N-2)*0.001;
uyk2(1,:) = linspace(1,1,N-2)*0.001;
uzk2(1,:) = linspace(1,1,N-2)*0.001;

% Xk3(1,:) = linspace(x03,xf3,N);
% Yk3(1,:) = linspace(y03,yf3,N);
Xk3(1,:) = interp1([1,fix(N/2),N],[x03,xf3,xf3],1:N,'linear','extrap');
Yk3(1,:) = interp1([1,fix(N/2),N],[y03,0,yf3],1:N,'linear','extrap');
Zk3(1,:) = linspace(z03,zf3,N);
vxk3(1,:) = linspace(Vx_03,Vx_f3,N-1);
vyk3(1,:) = linspace(Vy_03,Vy_f3,N-1);
vzk3(1,:) = linspace(Vz_03,Vz_f3,N-1);
uxk3(1,:) = linspace(1,1,N-2)*0.001;
uyk3(1,:) = linspace(1,1,N-2)*0.001;
uzk3(1,:) = linspace(1,1,N-2)*0.001;

errorx0 = zeros(s,1); errory0 = zeros(s,1); errorz0 = zeros(s,1);
errorx1 = zeros(s,1); errory1 = zeros(s,1); errorz1 = zeros(s,1);
errorx2 = zeros(s,1); errory2 = zeros(s,1); errorz2 = zeros(s,1);
errorx3 = zeros(s,1); errory3 = zeros(s,1); errorz3 = zeros(s,1);
errort  = zeros(s,1);
saf01 = zeros(1,N);
saf02 = zeros(1,N);
saf03 = zeros(1,N);
saf12 = zeros(1,N);
saf13 = zeros(1,N);
saf23 = zeros(1,N);
F = [];
%%   %-----------------------------±ß½çÓëÔ¼Êø----------------------%
for s = 1:S-1
%     if s >=2 && errort(end) <= tol
%          Xk_temp = Xk(s-1,:);Yk_temp = Yk(s-1,:);Zk_temp = Zk(s-1,:);
%     else
%         Xk_temp = Xk(s,:);Yk_temp = Yk(s,:);Zk_temp = Zk(s,:);
%     end
    Xk0_temp = Xk0(s,:);Yk0_temp = Yk0(s,:);Zk0_temp = Zk0(s,:);
    Xk1_temp = Xk1(s,:);Yk1_temp = Yk1(s,:);Zk1_temp = Zk1(s,:);
    Xk2_temp = Xk2(s,:);Yk2_temp = Yk2(s,:);Zk2_temp = Zk2(s,:);    
    Xk3_temp = Xk3(s,:);Yk3_temp = Yk3(s,:);Zk3_temp = Zk3(s,:);       
    for oci = 1:length(Roc)
        for nn = 1:N
            gc00(nn,oci)  = [(Xk0_temp(nn) - Xoc(oci));(Yk0_temp(nn) - Yoc(oci))]'*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk0_temp(nn) - Xoc(oci));(Yk0_temp(nn) - Yoc(oci))]  - Roc(oci)^2;
            Dgc0(:,nn,oci)  = 2*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk0_temp(nn)-Xoc(oci));(Yk0_temp(nn)-Yoc(oci))];
            lc0 = Roc(oci)*[Xk0_temp(nn)-Xoc(oci);Yk0_temp(nn)-Yoc(oci)]/norm( inv(Aoc{oci,1})*[Xk0_temp(nn)-Xoc(oci);Yk0_temp(nn)-Yoc(oci)]);
            lmove0(nn,oci) = ( gc00(nn,oci) + Dgc0(:,nn,oci)'*(lc0 + [Xoc(oci);Yoc(oci)]-[Xk0_temp(nn);Yk0_temp(nn)]));
            
            gc01(nn,oci)  = [(Xk1_temp(nn) - Xoc(oci));(Yk1_temp(nn) - Yoc(oci))]'*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk1_temp(nn) - Xoc(oci));(Yk1_temp(nn) - Yoc(oci))]  - Roc(oci)^2;
            Dgc1(:,nn,oci)  = 2*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk1_temp(nn)-Xoc(oci));(Yk1_temp(nn)-Yoc(oci))];
            lc1 = Roc(oci)*[Xk1_temp(nn)-Xoc(oci);Yk1_temp(nn)-Yoc(oci)]/norm( inv(Aoc{oci,1})*[Xk1_temp(nn)-Xoc(oci);Yk1_temp(nn)-Yoc(oci)]);
            lmove1(nn,oci) = ( gc01(nn,oci) + Dgc1(:,nn,oci)'*(lc1 + [Xoc(oci);Yoc(oci)]-[Xk1_temp(nn);Yk1_temp(nn)]));
            
            gc02(nn,oci)  = [(Xk2_temp(nn) - Xoc(oci));(Yk2_temp(nn) - Yoc(oci))]'*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk2_temp(nn) - Xoc(oci));(Yk2_temp(nn) - Yoc(oci))]  - Roc(oci)^2;
            Dgc2(:,nn,oci)  = 2*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk2_temp(nn)-Xoc(oci));(Yk2_temp(nn)-Yoc(oci))];
            lc2 = Roc(oci)*[Xk2_temp(nn)-Xoc(oci);Yk2_temp(nn)-Yoc(oci)]/norm( inv(Aoc{oci,1})*[Xk2_temp(nn)-Xoc(oci);Yk2_temp(nn)-Yoc(oci)]);
            lmove2(nn,oci) = ( gc02(nn,oci) + Dgc2(:,nn,oci)'*(lc2 + [Xoc(oci);Yoc(oci)]-[Xk2_temp(nn);Yk2_temp(nn)]));            

            gc03(nn,oci)  = [(Xk3_temp(nn) - Xoc(oci));(Yk3_temp(nn) - Yoc(oci))]'*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk3_temp(nn) - Xoc(oci));(Yk3_temp(nn) - Yoc(oci))]  - Roc(oci)^2;
            Dgc3(:,nn,oci)  = 2*inv(Aoc{oci,1})'/(Aoc{oci,1})*[(Xk3_temp(nn)-Xoc(oci));(Yk3_temp(nn)-Yoc(oci))];
            lc3 = Roc(oci)*[Xk3_temp(nn)-Xoc(oci);Yk3_temp(nn)-Yoc(oci)]/norm( inv(Aoc{oci,1})*[Xk3_temp(nn)-Xoc(oci);Yk3_temp(nn)-Yoc(oci)]);
            lmove3(nn,oci) = ( gc03(nn,oci) + Dgc3(:,nn,oci)'*(lc3 + [Xoc(oci);Yoc(oci)]-[Xk3_temp(nn);Yk3_temp(nn)]));              
            
        end
    end

% Collision Avoidance Constraints:Two-variable liearization function    

%     for nn = 1:N
%         r0 = [Xk0_temp(nn);Yk0_temp(nn);Zk0_temp(nn)];
%         r1 = [Xk1_temp(nn);Yk1_temp(nn);Zk1_temp(nn)];
%         r2 = [Xk2_temp(nn);Yk2_temp(nn);Zk2_temp(nn)];
%         r3 = [Xk3_temp(nn);Yk3_temp(nn);Zk3_temp(nn)];
%         C0_01(1,nn) = -(r0-r1)'*(r0-r1)-Rca^2;
%         C1_01(:,nn) = 2*(r0-r1);
%         C2_01(:,nn) = -2*(r0-r1);
%         C0_02(:,nn) = -(r0-r2)'*(r0-r2)-Rca^2;
%         C1_02(:,nn) = 2*(r0-r2);
%         C2_02(:,nn) = -2*(r0-r2); 
%         C0_03(1,nn) = -(r0-r3)'*(r0-r3)-Rca^2;
%         C1_03(:,nn) = 2*(r0-r3);
%         C2_03(:,nn) = -2*(r0-r3);
%         C0_12(1,nn) = -(r1-r2)'*(r1-r2)-Rca^2;
%         C1_12(:,nn) = 2*(r1-r2);
%         C2_12(:,nn) = -2*(r1-r2);
%         C0_13(1,nn) = -(r1-r3)'*(r1-r3)-Rca^2;
%         C1_13(:,nn) = 2*(r1-r3);
%         C2_13(:,nn) = -2*(r1-r3);               
%         C0_23(1,nn) = -(r2-r3)'*(r2-r3)-Rca^2;
%         C1_23(:,nn) = 2*(r2-r3);
%         C2_23(:,nn) = -2*(r2-r3);                 
%     end
    
    
    x0  = sdpvar(1,N);
    y0  = sdpvar(1,N);
    z0  = sdpvar(1,N);
    vx0 = sdpvar(1,N-1);
    vy0 = sdpvar(1,N-1);
    vz0 = sdpvar(1,N-1);
    ux0 = sdpvar(1,N-2);
    uy0 = sdpvar(1,N-2);
    uz0 = sdpvar(1,N-2);
    
    x1  = sdpvar(1,N);
    y1  = sdpvar(1,N);
    z1  = sdpvar(1,N);
    vx1 = sdpvar(1,N-1);
    vy1 = sdpvar(1,N-1);
    vz1 = sdpvar(1,N-1);
    ux1 = sdpvar(1,N-2);
    uy1 = sdpvar(1,N-2);
    uz1 = sdpvar(1,N-2);   
    
    x2  = sdpvar(1,N);
    y2  = sdpvar(1,N);
    z2  = sdpvar(1,N);
    vx2 = sdpvar(1,N-1);
    vy2 = sdpvar(1,N-1);
    vz2 = sdpvar(1,N-1);
    ux2 = sdpvar(1,N-2);
    uy2 = sdpvar(1,N-2);
    uz2 = sdpvar(1,N-2);     
    
    x3  = sdpvar(1,N);
    y3  = sdpvar(1,N);
    z3  = sdpvar(1,N);
    vx3 = sdpvar(1,N-1);
    vy3 = sdpvar(1,N-1);
    vz3 = sdpvar(1,N-1);
    ux3 = sdpvar(1,N-2);
    uy3 = sdpvar(1,N-2);
    uz3 = sdpvar(1,N-2);
    
%     s0 = sdpvar(1,N);
%     s1 = sdpvar(1,N);
%     s2 = sdpvar(1,N);
%     s3 = sdpvar(1,N);    
    
    T1  = sdpvar(1,1);
    T2  = sdpvar(1,1);
    
    F  = [  x0(1,1) == x00];
    F  = F +  [y0(1,1) == y00];    
    F  = F +  [z0(1,1) == z00];    
    F  = F +  [vx0(1,1) == Vx_00*T1];
    F  = F +  [vy0(1,1) == Vy_00*T1];
    F  = F +  [vz0(1,1) == Vz_00*T1];   
    
    F  = F +  [x1(1,1) == x01];
    F  = F +  [y1(1,1) == y01];    
    F  = F +  [z1(1,1) == z01];    
    F  = F +  [vx1(1,1) == Vx_01*T1];
    F  = F +  [vy1(1,1) == Vy_01*T1];
    F  = F +  [vz1(1,1) == Vz_01*T1];  
    
    F  = F +  [x2(1,1) == x02];
    F  = F +  [y2(1,1) == y02];    
    F  = F +  [z2(1,1) == z02];    
    F  = F +  [vx2(1,1) == Vx_02*T1];
    F  = F +  [vy2(1,1) == Vy_02*T1];
    F  = F +  [vz2(1,1) == Vz_02*T1];     
    
    F  = F +  [x3(1,1) == x03];
    F  = F +  [y3(1,1) == y03];    
    F  = F +  [z3(1,1) == z03];    
    F  = F +  [vx3(1,1) == Vx_03*T1];
    F  = F +  [vy3(1,1) == Vy_03*T1];
    F  = F +  [vz3(1,1) == Vz_03*T1];         
    
    
%     F  = F +  [x0(1,N) == x1(1,N)];
%     F  = F +  [y0(1,N) == y1(1,N)];
%     F  = F +  [z0(1,N) == z1(1,N)];
    F  = F +  [x0(1,N) == xf0];
    F  = F +  [y0(1,N) == yf0];
    F  = F +  [z0(1,N) == zf0];  
    F  = F +  [vx0(1,N-1) == Vx_f0*T1];
    F  = F +  [vy0(1,N-1) == Vy_f0*T1];
    F  = F +  [vz0(1,N-1) == Vz_f0*T1];      
%     F  = F +  [vx0(1,N-1) == vx1(1,N-1)];
%     F  = F +  [vy0(1,N-1) == vy1(1,N-1)];
%     F  = F +  [vz0(1,N-1) == vz1(1,N-1)];  
    
%     F  = F +  [x1(1,N) == x2(1,N)];
%     F  = F +  [y1(1,N) == y2(1,N)];
%     F  = F +  [z1(1,N) == z2(1,N)];
    F  = F +  [x1(1,N) == xf1];
    F  = F +  [y1(1,N) == yf1];
    F  = F +  [z1(1,N) == zf1];  
    F  = F +  [vx1(1,N-1) == Vx_f1*T1];
    F  = F +  [vy1(1,N-1) == Vy_f1*T1];
    F  = F +  [vz1(1,N-1) == Vz_f1*T1];     
%     F  = F +  [vx1(1,N-1) == vx2(1,N-1)];
%     F  = F +  [vy1(1,N-1) == vy2(1,N-1)];
%     F  = F +  [vz1(1,N-1) == vz2(1,N-1)];      
    
%     F  = F +  [x2(1,N) == x3(1,N)];
%     F  = F +  [y2(1,N) == y3(1,N)];
%     F  = F +  [z2(1,N) == z3(1,N)];
    F  = F +  [x2(1,N) == xf2];
    F  = F +  [y2(1,N) == yf2];
    F  = F +  [z2(1,N) == zf2];    
    F  = F +  [vx2(1,N-1) == Vx_f2*T1];
    F  = F +  [vy2(1,N-1) == Vy_f2*T1];
    F  = F +  [vz2(1,N-1) == Vz_f2*T1];      
%     F  = F +  [vx2(1,N-1) == vx3(1,N-1)];
%     F  = F +  [vy2(1,N-1) == vy3(1,N-1)];
%     F  = F +  [vz2(1,N-1) == vz3(1,N-1)];     
%     
    F  = F +  [x3(1,N) == xf3];
    F  = F +  [y3(1,N) == yf3];
    F  = F +  [z3(1,N) == zf3];         
    F  = F +  [vx3(1,N-1) == Vx_f3*T1];
    F  = F +  [vy3(1,N-1) == Vy_f3*T1];
    F  = F +  [vz3(1,N-1) == Vz_f3*T1];    
    
    
    
%     F  = F +  [10<= z0(1,N)];
%     F  = F +  [vz0(1,N-1) == 0];      

    F  = F +  [.0 <= T1];
    F  = F +  [.0 <= T2];
    
    for i = 1:N-1
        % First-order Euler method
        F  = F + [x0(1,i+1) == x0(1,i) + tau*vx0(1,i)];
        F  = F + [y0(1,i+1) == y0(1,i) + tau*vy0(1,i)];
        F  = F + [z0(1,i+1) == z0(1,i) + tau*vz0(1,i)];
        
        F  = F + [x1(1,i+1) == x1(1,i) + tau*vx1(1,i)];
        F  = F + [y1(1,i+1) == y1(1,i) + tau*vy1(1,i)];
        F  = F + [z1(1,i+1) == z1(1,i) + tau*vz1(1,i)];       
        
        F  = F + [x2(1,i+1) == x2(1,i) + tau*vx2(1,i)];
        F  = F + [y2(1,i+1) == y2(1,i) + tau*vy2(1,i)];
        F  = F + [z2(1,i+1) == z2(1,i) + tau*vz2(1,i)];      
        
        F  = F + [x3(1,i+1) == x3(1,i) + tau*vx3(1,i)];
        F  = F + [y3(1,i+1) == y3(1,i) + tau*vy3(1,i)];
        F  = F + [z3(1,i+1) == z3(1,i) + tau*vz3(1,i)];            
        
%         F  = F + [Vzmin*T1 <= vz(1,i) <= Vzmax*T1];             % linear constraints for As/Descent speed
%         F  = F + [ norm([vx(1,i);vy(1,i)]) <= Vlmax*T1];        % cone constraints for level speed        
        F  = F + [ norm([vx0(1,i);vy0(1,i);vz0(1,i)]) <= Speedmax0*T1];  %second-order cone for velocity
        F  = F + [ norm([vx1(1,i);vy1(1,i);vz1(1,i)]) <= Speedmax1*T1];  %second-order cone for velocity     
        F  = F + [ norm([vx2(1,i);vy2(1,i);vz2(1,i)]) <= Speedmax2*T1];  %second-order cone for velocity  
        F  = F + [ norm([vx3(1,i);vy3(1,i);vz3(1,i)]) <= Speedmax3*T1];  %second-order cone for velocity        

%         F  = F + [ norm([vx(1,i);vy(1,i);vz(1,i)])^2 <= Vmax(1,i)*T2]; %rotate  cone for velocity
    end
    
    for j = 1:N-2
        F  = F + [vx0(1,j+1) == vx0(1,j) + tau*ux0(1,j)];
        F  = F + [vy0(1,j+1) == vy0(1,j) + tau*uy0(1,j)];
        F  = F + [vz0(1,j+1) == vz0(1,j) + tau*(uz0(1,j) - g*T2/anorm)];
        
        F  = F + [vx1(1,j+1) == vx1(1,j) + tau*ux1(1,j)];
        F  = F + [vy1(1,j+1) == vy1(1,j) + tau*uy1(1,j)];
        F  = F + [vz1(1,j+1) == vz1(1,j) + tau*(uz1(1,j) - g*T2/anorm)];     
        
        F  = F + [vx2(1,j+1) == vx2(1,j) + tau*ux2(1,j)];
        F  = F + [vy2(1,j+1) == vy2(1,j) + tau*uy2(1,j)];
        F  = F + [vz2(1,j+1) == vz2(1,j) + tau*(uz2(1,j) - g*T2/anorm)];      
        
        F  = F + [vx3(1,j+1) == vx3(1,j) + tau*ux3(1,j)];
        F  = F + [vy3(1,j+1) == vy3(1,j) + tau*uy3(1,j)];
        F  = F + [vz3(1,j+1) == vz3(1,j) + tau*(uz3(1,j) - g*T2/anorm)];           
        
    end
    
    
    for ui = 1:N-2
        F  = F + [norm([ux0(1,ui);uy0(1,ui);uz0(1,ui)]) <= amax*T2];
%         F  = F + [norm([ux0(1,ui);uy0(1,ui)]) <= tan(Tiltmax)*uz0(1,ui)]; %Attitude angle constraint.
%         F  = F + [T2*uzmin <= uz0(1,ui)]; %Approximate minimum z acceleration constraint.   

        F  = F + [norm([ux1(1,ui);uy1(1,ui);uz1(1,ui)]) <= amax*T2];
%         F  = F + [norm([ux1(1,ui);uy1(1,ui)]) <= tan(Tiltmax)*uz1(1,ui)]; %Attitude angle constraint.
%         F  = F + [T2*uzmin <= uz1(1,ui)]; %Approximate minimum z acceleration constraint.  

        F  = F + [norm([ux2(1,ui);uy2(1,ui);uz2(1,ui)]) <= amax*T2];
%         F  = F + [norm([ux2(1,ui);uy2(1,ui)]) <= tan(Tiltmax)*uz2(1,ui)]; %Attitude angle constraint.
%         F  = F + [T2*uzmin <= uz2(1,ui)]; %Approximate minimum z acceleration constraint.  

        F  = F + [norm([ux3(1,ui);uy3(1,ui);uz3(1,ui)]) <= amax*T2];
%         F  = F + [norm([ux3(1,ui);uy3(1,ui)]) <= tan(Tiltmax)*uz3(1,ui)]; %Attitude angle constraint.
%         F  = F + [T2*uzmin <= uz3(1,ui)]; %Approximate minimum z acceleration constraint.  

    end
    
    F  = F + rcone(T1,T2,0.5);
    
    if flagobs == 1
        for ocj = 1:length(Roc)
            for oi = 1:N
                switch method
                    case 'Linearization'
                        F  = [F; gc00(oi,ocj)+Dgc0(:,oi,ocj)'*[(x0(oi)-Xk0_temp(oi));(y0(oi)-Yk0_temp(oi))]>= 0];
                        F  = [F; gc01(oi,ocj)+Dgc1(:,oi,ocj)'*[(x1(oi)-Xk1_temp(oi));(y1(oi)-Yk1_temp(oi))]>= 0];         
                        F  = [F; gc02(oi,ocj)+Dgc2(:,oi,ocj)'*[(x2(oi)-Xk2_temp(oi));(y2(oi)-Yk2_temp(oi))]>= 0];
                        F  = [F; gc03(oi,ocj)+Dgc3(:,oi,ocj)'*[(x3(oi)-Xk3_temp(oi));(y3(oi)-Yk3_temp(oi))]>= 0];    
%                         F  = [F; gc00(oi,ocj)+Dgc0(:,oi,ocj)'*[(x0(oi)-Xk0_temp(oi));(y0(oi)-Yk0_temp(oi))] >=  -s0(oi)];
%                         F  = [F;s0(oi) >= 0];                        
%                         F  = [F; gc01(oi,ocj)+Dgc1(:,oi,ocj)'*[(x1(oi)-Xk1_temp(oi));(y1(oi)-Yk1_temp(oi))] >=  -s1(oi)];
%                         F  = [F;s1(oi) >= 0];      
%                         F  = [F; gc02(oi,ocj)+Dgc2(:,oi,ocj)'*[(x2(oi)-Xk2_temp(oi));(y2(oi)-Yk2_temp(oi))] >=  -s2(oi)];
%                         F  = [F;s2(oi) >= 0];    
%                         F  = [F; gc03(oi,ocj)+Dgc3(:,oi,ocj)'*[(x3(oi)-Xk3_temp(oi));(y3(oi)-Yk3_temp(oi))] >=  -s3(oi)];
%                         F  = [F;s3(oi) >= 0];                               
                    case 'Projection'
                        % nonlinear past the no-fly zones
%                         F  = [F; gc00(oi,ocj)-lmove0(oi,ocj)+Dgc0(:,oi,ocj)'*[(x0(oi)-Xk0_temp(oi));(y0(oi)-Yk0_temp(oi))] >=  -s0(oi)];
%                         F  = [F;s0(oi) >= 0];                        
%                         F  = [F; gc01(oi,ocj)-lmove1(oi,ocj)+Dgc1(:,oi,ocj)'*[(x1(oi)-Xk1_temp(oi));(y1(oi)-Yk1_temp(oi))] >=  -s1(oi)];
%                         F  = [F;s1(oi) >= 0];      
%                         F  = [F; gc02(oi,ocj)-lmove2(oi,ocj)+Dgc2(:,oi,ocj)'*[(x2(oi)-Xk2_temp(oi));(y2(oi)-Yk2_temp(oi))] >=  -s2(oi)];
%                         F  = [F;s2(oi) >= 0];    
%                         F  = [F; gc03(oi,ocj)-lmove3(oi,ocj)+Dgc3(:,oi,ocj)'*[(x3(oi)-Xk3_temp(oi));(y3(oi)-Yk3_temp(oi))] >=  -s3(oi)];
%                         F  = [F;s3(oi) >= 0];                                                    
                        F  = [F; gc00(oi,ocj)-lmove0(oi,ocj)+Dgc0(:,oi,ocj)'*[(x0(oi)-Xk0_temp(oi));(y0(oi)-Yk0_temp(oi))]>= 0];
                        F  = [F; gc01(oi,ocj)-lmove1(oi,ocj)+Dgc1(:,oi,ocj)'*[(x1(oi)-Xk1_temp(oi));(y1(oi)-Yk1_temp(oi))]>= 0];                        
                        F  = [F; gc02(oi,ocj)-lmove2(oi,ocj)+Dgc2(:,oi,ocj)'*[(x2(oi)-Xk2_temp(oi));(y2(oi)-Yk2_temp(oi))]>= 0];  
                        F  = [F; gc03(oi,ocj)-lmove3(oi,ocj)+Dgc3(:,oi,ocj)'*[(x3(oi)-Xk3_temp(oi));(y3(oi)-Yk3_temp(oi))]>= 0];    

%                         %    F  = [F; gc0(oi,ocj)-pc(oi,ocj)+Dgc(:,oi,ocj)'*[(x(oi)-Xk_temp(oi));(y(oi)-Yk_temp(oi))]>= 0];
                end
            end
        end
    end
    
    % Collision Avoidance Constraints
    if flagcol == 1 && s >= 4
%         for oi = 1:N
%             F = [F;C0_01(1,oi)+C1_01(:,oi)'*[x0(oi);y0(oi);z0(oi)]+C2_01(:,oi)'*[x1(oi);y1(oi);z1(oi)] >= 0];
%             F = [F;C0_02(1,oi)+C1_02(:,oi)'*[x0(oi);y0(oi);z0(oi)]+C2_02(:,oi)'*[x2(oi);y2(oi);z2(oi)] >= 0];
%             F = [F;C0_03(1,oi)+C1_03(:,oi)'*[x0(oi);y0(oi);z0(oi)]+C2_03(:,oi)'*[x3(oi);y3(oi);z3(oi)] >= 0];
%             F = [F;C0_12(1,oi)+C1_12(:,oi)'*[x1(oi);y1(oi);z1(oi)]+C2_12(:,oi)'*[x2(oi);y2(oi);z2(oi)] >= 0];
%             F = [F;C0_13(1,oi)+C1_13(:,oi)'*[x1(oi);y1(oi);z1(oi)]+C2_13(:,oi)'*[x3(oi);y3(oi);z3(oi)] >= 0];
%             F = [F;C0_23(1,oi)+C1_23(:,oi)'*[x2(oi);y2(oi);z2(oi)]+C2_23(:,oi)'*[x3(oi);y3(oi);z3(oi)] >= 0];
%         end
        for oi = 1:N
            r0 = [Xk0_temp(oi);Yk0_temp(oi);Zk0_temp(oi)];
            r1 = [Xk1_temp(oi);Yk1_temp(oi);Zk1_temp(oi)];
            r2 = [Xk2_temp(oi);Yk2_temp(oi);Zk2_temp(oi)];
            r3 = [Xk3_temp(oi);Yk3_temp(oi);Zk3_temp(oi)];
            % Missile 0            
            if norm(r0-r1,2)<=Rca || norm(r0-r2,2)<=Rca || norm(r0-r3,2)<=Rca
                F = [F;(r0-r1)'*(r0-r1)-Rca^2 + 2*(r0-r1)'*([x0(oi);y0(oi);z0(oi)]-r0) >= 0];
                F = [F;(r0-r2)'*(r0-r2)-Rca^2 + 2*(r0-r2)'*([x0(oi);y0(oi);z0(oi)]-r0) >= 0];
                F = [F;(r0-r3)'*(r0-r3)-Rca^2 + 2*(r0-r3)'*([x0(oi);y0(oi);z0(oi)]-r0) >= 0];
            end
            % missile 1
            if norm(r1-r2,2)<=Rca || norm(r1-r3,2)<=Rca || norm(r1-r0,2)<=Rca
                F = [F;(r1-r2)'*(r1-r2)-Rca^2 + 2*(r1-r2)'*([x1(oi);y1(oi);z1(oi)]-r1) >= 0];
                F = [F;(r1-r3)'*(r1-r3)-Rca^2 + 2*(r1-r3)'*([x1(oi);y1(oi);z1(oi)]-r1) >= 0];
                F = [F;(r1-r0)'*(r1-r0)-Rca^2 + 2*(r1-r0)'*([x1(oi);y1(oi);z1(oi)]-r1) >= 0];    
            end
            % missile 2
            if norm(r2-r0,2)<=Rca || norm(r2-r1,2)<=Rca || norm(r2-r3,2)<=Rca 
                F = [F;(r2-r0)'*(r2-r0)-Rca^2 + 2*(r2-r0)'*([x2(oi);y2(oi);z2(oi)]-r2) >= 0];
                F = [F;(r2-r1)'*(r2-r1)-Rca^2 + 2*(r2-r1)'*([x2(oi);y2(oi);z2(oi)]-r2) >= 0];
                F = [F;(r2-r3)'*(r2-r3)-Rca^2 + 2*(r2-r3)'*([x2(oi);y2(oi);z2(oi)]-r2) >= 0]; 
            end
            % missile 3            
            if norm(r3-r0,2)<=Rca || norm(r3-r1,2)<=Rca || norm(r3-r2,2)<=Rca            
                F = [F;(r3-r0)'*(r3-r0)-Rca^2 + 2*(r3-r0)'*([x3(oi);y3(oi);z3(oi)]-r3) >= 0];
                F = [F;(r3-r1)'*(r3-r1)-Rca^2 + 2*(r3-r1)'*([x3(oi);y3(oi);z3(oi)]-r3) >= 0];
                F = [F;(r3-r2)'*(r3-r2)-Rca^2 + 2*(r3-r2)'*([x3(oi);y3(oi);z3(oi)]-r3) >= 0];  
            end
        end                        
    end
    
    
    ops = sdpsettings('solver','ecos','verbose',1);%mosek ipopt ecos
    % no sum(s2),sum(s3), path2,3 will across inside.
%     obj = T2 - Lambda(s)*T1 + 100*sum(s0) + 100*sum(s1) + 100*sum(s2) + 100*sum(s3);    
    %     obj = T2 - 0.5*Lambda(s)*T1;
        obj = T2 - Lambda(s)*T1;
%     obj = T2 - 0.3*14.4749386260*T1;     
    %     if s >= 12
    %         obj = T2 - Lambda(6)*T1;
    %     else
    %         obj = T2 - 0.3*Lambda(s)*T1;
    %     end
    solvesdp(F, obj, ops)
    TT2  = double(T2);
    TT1  = double(T1);
    actierror = TT1^2-TT2;
    X0   = double(x0);
    Y0   = double(y0);
    Z0   = double(z0);
    Vx0   = double(vx0);
    Vy0   = double(vy0);
    Vz0   = double(vz0);
    Ux0   = double(ux0);
    Uy0   = double(uy0);
    Uz0   = double(uz0);
    
    X1   = double(x1);
    Y1   = double(y1);
    Z1   = double(z1);
    Vx1   = double(vx1);
    Vy1   = double(vy1);
    Vz1   = double(vz1);
    Ux1   = double(ux1);
    Uy1   = double(uy1);
    Uz1   = double(uz1);    
    
    X2   = double(x2);
    Y2   = double(y2);
    Z2   = double(z2);
    Vx2   = double(vx2);
    Vy2   = double(vy2);
    Vz2   = double(vz2);
    Ux2   = double(ux2);
    Uy2   = double(uy2);
    Uz2   = double(uz2);    
    
    X3   = double(x3);
    Y3   = double(y3);
    Z3   = double(z3);
    Vx3   = double(vx3);
    Vy3   = double(vy3);
    Vz3   = double(vz3);
    Ux3   = double(ux3);
    Uy3   = double(uy3);
    Uz3   = double(uz3);      
    
    Fobj(s) = double(obj);

    Xk0(s+1,1:N) = X0; Yk0(s+1,1:N) = Y0; Zk0(s+1,1:N) = Z0;
    vxk0(s+1,:) = Vx0; vyk0(s+1,:) = Vy0; vzk0(s+1,:) = Vz0;
    uxk0(s+1,:) = Ux0; uyk0(s+1,:) = Uy0; uzk0(s+1,:) = Uz0;
    
    Xk1(s+1,1:N) = X1; Yk1(s+1,1:N) = Y1; Zk1(s+1,1:N) = Z1;
    vxk1(s+1,:) = Vx1; vyk1(s+1,:) = Vy1; vzk1(s+1,:) = Vz1;
    uxk1(s+1,:) = Ux1; uyk1(s+1,:) = Uy1; uzk1(s+1,:) = Uz1;    

    Xk2(s+1,1:N) = X2; Yk2(s+1,1:N) = Y2; Zk2(s+1,1:N) = Z2;
    vxk2(s+1,:) = Vx2; vyk2(s+1,:) = Vy2; vzk2(s+1,:) = Vz2;
    uxk2(s+1,:) = Ux2; uyk2(s+1,:) = Uy2; uzk2(s+1,:) = Uz2;   
    
    Xk3(s+1,1:N) = X3; Yk3(s+1,1:N) = Y3; Zk3(s+1,1:N) = Z3;
    vxk3(s+1,:) = Vx3; vyk3(s+1,:) = Vy3; vzk3(s+1,:) = Vz3;
    uxk3(s+1,:) = Ux3; uyk3(s+1,:) = Uy3; uzk3(s+1,:) = Uz3;       
    
    T2k(s+1,1) = TT2;T1k(s+1,1) = TT1;
    errorx0(s) = max(abs(Xk0(s+1,:)-Xk0(s,:)));
    errory0(s) = max(abs(Yk0(s+1,:)-Yk0(s,:)));
    errorz0(s) = max(abs(Zk0(s+1,:)-Zk0(s,:)));
    errorx1(s) = max(abs(Xk1(s+1,:)-Xk1(s,:)));
    errory1(s) = max(abs(Yk1(s+1,:)-Yk1(s,:)));
    errorz1(s) = max(abs(Zk1(s+1,:)-Zk1(s,:)));    
    errorx2(s) = max(abs(Xk2(s+1,:)-Xk2(s,:)));
    errory2(s) = max(abs(Yk2(s+1,:)-Yk2(s,:)));
    errorz2(s) = max(abs(Zk2(s+1,:)-Zk2(s,:)));    
    errorx3(s) = max(abs(Xk3(s+1,:)-Xk3(s,:)));
    errory3(s) = max(abs(Yk3(s+1,:)-Yk3(s,:)));
    errorz3(s) = max(abs(Zk3(s+1,:)-Zk3(s,:)));     
    flagerror0 = (errorx0(s) <= tol) && (errory0(s) <= tol)&& (errorz0(s) <= tol);
    flagerror1 = (errorx1(s) <= tol) && (errory1(s) <= tol)&& (errorz1(s) <= tol);  
    flagerror2 = (errorx2(s) <= tol) && (errory2(s) <= tol)&& (errorz2(s) <= tol);  
    flagerror3 = (errorx3(s) <= tol) && (errory3(s) <= tol)&& (errorz3(s) <= tol);      
    errort(s) = abs(T1k(s+1,:)-T1k(s,:));   
    

for i = 1:N
    saf01(i) = norm([Xk0(end,i)-Xk1(end,i);Yk0(end,i)-Yk1(end,i);Zk0(end,i)-Zk1(end,i)],2);
    saf02(i) = norm([Xk0(end,i)-Xk2(end,i);Yk0(end,i)-Yk2(end,i);Zk0(end,i)-Zk2(end,i)],2);   
    saf03(i) = norm([Xk0(end,i)-Xk3(end,i);Yk0(end,i)-Yk3(end,i);Zk0(end,i)-Zk3(end,i)],2);     
    saf12(i) = norm([Xk1(end,i)-Xk2(end,i);Yk1(end,i)-Yk2(end,i);Zk1(end,i)-Zk2(end,i)],2);
    saf13(i) = norm([Xk1(end,i)-Xk3(end,i);Yk1(end,i)-Yk3(end,i);Zk1(end,i)-Zk3(end,i)],2);   
    saf23(i) = norm([Xk2(end,i)-Xk3(end,i);Yk2(end,i)-Yk3(end,i);Zk2(end,i)-Zk3(end,i)],2);      
end    
flagsaf = (min(saf01) >= Rca) && (min(saf02) >= Rca) && (min(saf03) >= Rca) && (min(saf12) >= Rca) ...
    && (min(saf13) >= Rca) && (min(saf23) >= Rca);
    
%     if errorx1(s) <= tol && errory1(s) <= tol && errorz1(s) <= tol || flagobs == 0
    if flagerror0 || flagerror1 || flagerror2 || flagerror3 || flagobs == 0        
        q_a = 1;
        q_b = -Lambda(s);
        q_c = Lambda(s)*TT1-TT2;
        T1_inters = 0.5*(-q_b+sqrt(q_b^2-4*q_a*q_c))/q_a;
        Lambda(s+1,:)= 2*T1_inters;%2*sqrt(T1_inters)
    else
        Lambda(s+1,:)=Lambda(s,:);
    end
    fprintf('SCOP completed %d times optimization,the optimal flight time is %d seconds, cvx_error is %d.\n ',...
        s,sqrt(TT1)*Tnorm,actierror*Tnorm);
    fprintf('the errorx, errory and errorz are %d m, %d m and %d m.\n',...
        errorx0(s)*Lnorm,errory0(s)*Lnorm,errorz0(s)*Lnorm);
    fprintf('the errort is %d s.\n',errort(s)*Tnorm);
    %%%
%     if errorx0(s) <= tol && errory0(s) <= tol && errorz0(s) <= tol...
%             && errort(s) <= tol && abs(actierror) <=tol && flagobs == 1
%         break
%     elseif abs(actierror) <=tol && flagobs == 0
%         break
%     end
    if (flagerror0 || flagerror1 || flagerror2 || flagerror3) && ...
            flagsaf && errort(s) <= tol && abs(actierror) <=tol && flagobs == 1
        break
    elseif abs(actierror) <=tol && flagobs == 0
        break
    end    
    figure(11)
    plot3(Xk0(end,:),Yk0(end,:),Zk0(end,:),'--b','Linewidth',1);
    xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
    hold on;grid on;
    time =linspace(0,tau*N,N)*TT1*Tnorm;
    Vv = sqrt(vxk0(end,:).^2 + vyk0(end,:).^2 + vzk0(end,:).^2)/TT1;
    Vlev = sqrt(vxk0(end,:).^2 + vyk0(end,:).^2 )/TT1;
    figure(12)
    subplot(2,1,1)
    plot(time(1:end-1),Vv*Vnorm,'Linewidth',2);
    xlabel('time(s)');ylabel('V(m/s)');hold on;
    subplot(2,1,2)
    plot(time(1:end-1),Vlev*Vnorm,'Linewidth',2);
    xlabel('time(s)');ylabel('Vlev(m/s)');hold on;
    figure(22)
    subplot(3,1,1)
    plot(time(1:end-1),Vx0*Vnorm,'Linewidth',2);
    xlabel('time(s)');ylabel('V_x(m/s)');hold on;
    subplot(3,1,2)
    plot(time(1:end-1),Vy0*Vnorm,'Linewidth',2);
    xlabel('time(s)');ylabel('V_y(m/s)');hold on;
    subplot(3,1,3)
    plot(time(1:end-1),Vz0*Vnorm,'Linewidth',2);
    xlabel('time(s)');ylabel('V_z(m/s)');hold on;    
    
end
%%
close all
for i = 1:N-2
    ux_norm=(Ux0/TT2*anorm)./norm([Ux0/(TT2)*anorm,Uy0/(TT2)*anorm,Uz0/(TT2)*anorm],2);
    uy_norm=(Uy0/TT2*anorm)./norm([Ux0/(TT2)*anorm,Uy0/(TT2)*anorm,Uz0/(TT2)*anorm],2);
    uz_norm=(Uz0/TT2*anorm)./norm([Ux0/(TT2)*anorm,Uy0/(TT2)*anorm,Uz0/(TT2)*anorm],2);
end
figure(1)
siz = size(Xk0);
for i = 1:siz(1)-1
    plot3(Xk0(i,:)*Lnorm,Yk0(i,:)*Lnorm,Zk0(i,:)*Lnorm,'--b','Linewidth',1);
    hold on;grid on;
    plot3(Xk1(i,:)*Lnorm,Yk1(i,:)*Lnorm,Zk1(i,:)*Lnorm,'--b','Linewidth',1);
    hold on;grid on;    
    plot3(Xk2(i,:)*Lnorm,Yk2(i,:)*Lnorm,Zk2(i,:)*Lnorm,'--b','Linewidth',1);
    hold on;grid on;       
    plot3(Xk3(i,:)*Lnorm,Yk3(i,:)*Lnorm,Zk3(i,:)*Lnorm,'--b','Linewidth',1);
    hold on;grid on;               
end
plot3(Xk0(end,:)*Lnorm, Yk0(end,:)*Lnorm,Zk0(end,:)*Lnorm,'r','Linewidth',2);
hold on;
plot3(Xk1(end,:)*Lnorm, Yk1(end,:)*Lnorm,Zk1(end,:)*Lnorm,'r','Linewidth',2);
hold on;
plot3(Xk2(end,:)*Lnorm, Yk2(end,:)*Lnorm,Zk2(end,:)*Lnorm,'r','Linewidth',2);
hold on;
plot3(Xk3(end,:)*Lnorm, Yk3(end,:)*Lnorm,Zk3(end,:)*Lnorm,'r','Linewidth',2);
hold on;
if flagobs == 1
    ObstaclePlot;
end
%%
figure(2)
labt = round(linspace(0,TT1*Tnorm,10));
labx0 = interp1(time,Xk0(end,:)*Lnorm,labt,'linear','extrap');
laby0 = interp1(time,Yk0(end,:)*Lnorm,labt,'linear','extrap');
labz0 = interp1(time,Zk0(end,:)*Lnorm,labt,'linear','extrap');
labx1 = interp1(time,Xk1(end,:)*Lnorm,labt,'linear','extrap');
laby1 = interp1(time,Yk1(end,:)*Lnorm,labt,'linear','extrap');
labz1 = interp1(time,Zk1(end,:)*Lnorm,labt,'linear','extrap');
labx2 = interp1(time,Xk2(end,:)*Lnorm,labt,'linear','extrap');
laby2 = interp1(time,Yk2(end,:)*Lnorm,labt,'linear','extrap');
labz2 = interp1(time,Zk2(end,:)*Lnorm,labt,'linear','extrap');
labx3 = interp1(time,Xk3(end,:)*Lnorm,labt,'linear','extrap');
laby3 = interp1(time,Yk3(end,:)*Lnorm,labt,'linear','extrap');
labz3 = interp1(time,Zk3(end,:)*Lnorm,labt,'linear','extrap');

% h1 = plot3(Xk0(end,:)*Lnorm/1000,Yk0(end,:)*Lnorm/1000,Zk0(end,:)*Lnorm/1000,'k:x','Markersize',5,'Linewidth',1);hold on;
% h2 = plot3(Xk1(end,:)*Lnorm/1000,Yk1(end,:)*Lnorm/1000,Zk1(end,:)*Lnorm/1000,'kd--','Markersize',3,'Linewidth',1);hold on;
% h3 = plot3(Xk2(end,:)*Lnorm/1000,Yk2(end,:)*Lnorm/1000,Zk2(end,:)*Lnorm/1000,'k:.','Markersize',8,'Linewidth',1);hold on;
% h4 = plot3(Xk3(end,:)*Lnorm/1000,Yk3(end,:)*Lnorm/1000,Zk3(end,:)*Lnorm/1000,'k-.o','Markersize',3,'Linewidth',1);hold on;
h1 = plot3(Xk0(end,:)*Lnorm/1000,Yk0(end,:)*Lnorm/1000,Zk0(end,:)*Lnorm/1000,'Linewidth',1);hold on;
h2 = plot3(Xk1(end,:)*Lnorm/1000,Yk1(end,:)*Lnorm/1000,Zk1(end,:)*Lnorm/1000,'Linewidth',1);hold on;
h3 = plot3(Xk2(end,:)*Lnorm/1000,Yk2(end,:)*Lnorm/1000,Zk2(end,:)*Lnorm/1000,'Linewidth',1);hold on;
h4 = plot3(Xk3(end,:)*Lnorm/1000,Yk3(end,:)*Lnorm/1000,Zk3(end,:)*Lnorm/1000,'Linewidth',1);hold on;
xlabel('x(km)');ylabel('y(km)');zlabel('z(km)');
% quiver3(Xk0(end,3:end)*Lnorm,Yk0(end,3:end)*Lnorm,Zk0(end,3:end)*Lnorm,ux_norm,uy_norm,uz_norm,0.9,'b','Linewidth',0.5);
% quiver3(Xk1(end,3:end)*Lnorm,Yk1(end,3:end)*Lnorm,Zk1(end,3:end)*Lnorm,ux_norm,uy_norm,uz_norm,0.9,'b','Linewidth',0.5);
% quiver3(Xk2(end,3:end)*Lnorm,Yk2(end,3:end)*Lnorm,Zk2(end,3:end)*Lnorm,ux_norm,uy_norm,uz_norm,0.9,'b','Linewidth',0.5);
% quiver3(Xk3(end,3:end)*Lnorm,Yk3(end,3:end)*Lnorm,Zk3(end,3:end)*Lnorm,ux_norm,uy_norm,uz_norm,0.9,'b','Linewidth',0.5);
% text(Xk0(end,end)*Lnorm,Yk0(end,end)*Lnorm,Zk0(end,end)*Lnorm,num2str(TT1*Tnorm));
% for i = 1:length(labt)-1
%     text(labx0(i),laby0(i),labz0(i),num2str(labt(i)));
%     text(labx1(i),laby1(i),labz1(i),num2str(labt(i)));   
%     text(labx2(i),laby2(i),labz2(i),num2str(labt(i)));   
%     text(labx3(i),laby3(i),labz3(i),num2str(labt(i)));       
% end
% xlim([min(Xk0(end,:)*Lnorm) max(Xk0(end,:)*Lnorm)]*1.1);
% ylim([min(Yk1(end,:)*Lnorm) max(Yk1(end,:)*Lnorm)]*1.1);
% ylim([min(Yk2(end,:)*Lnorm) max(Yk2(end,:)*Lnorm)]*1.1);
axis equal;
grid on;
if flagobs == 1
    ObstaclePlot;
end
legend([h1,h2,h3,h4],'Missile-1','Missile-2','Missile-3','Missile-4','Location','NorthEast');
%%
saf01 = zeros(1,N);
saf02 = zeros(1,N);
saf03 = zeros(1,N);
saf12 = zeros(1,N);
saf13 = zeros(1,N);
saf23 = zeros(1,N);
for i = 1:N
    saf01(i) = norm([Xk0(end,i)-Xk1(end,i);Yk0(end,i)-Yk1(end,i);Zk0(end,i)-Zk1(end,i)],2);
    saf02(i) = norm([Xk0(end,i)-Xk2(end,i);Yk0(end,i)-Yk2(end,i);Zk0(end,i)-Zk2(end,i)],2);   
    saf03(i) = norm([Xk0(end,i)-Xk3(end,i);Yk0(end,i)-Yk3(end,i);Zk0(end,i)-Zk3(end,i)],2);     
    saf12(i) = norm([Xk1(end,i)-Xk2(end,i);Yk1(end,i)-Yk2(end,i);Zk1(end,i)-Zk2(end,i)],2);
    saf13(i) = norm([Xk1(end,i)-Xk3(end,i);Yk1(end,i)-Yk3(end,i);Zk1(end,i)-Zk3(end,i)],2);   
    saf23(i) = norm([Xk2(end,i)-Xk3(end,i);Yk2(end,i)-Yk3(end,i);Zk2(end,i)-Zk3(end,i)],2);      
end
figure(1324)
plot(time,saf01*Lnorm/1000,'r-.','Linewidth',2);hold on;
plot(time,saf02*Lnorm/1000,'g-.','Linewidth',2);hold on;
plot(time,saf03*Lnorm/1000,'b-.','Linewidth',2);hold on;
plot(time,saf12*Lnorm/1000,'c-.','Linewidth',2);hold on;
plot(time,saf13*Lnorm/1000,'m-.','Linewidth',2);hold on;
plot(time,saf23*Lnorm/1000,'k-.','Linewidth',2);hold on;
xlabel('time(s)');ylabel('distance(km)');grid on;
legend('s-12','s-13','s-14','s-23','s-24','s-34');
%%
figure(3)
subplot(3,1,1)
plot(time(1:N-1),Vx0(1:N-1)/TT1*Vnorm,'k','Linewidth',2);
xlabel('time(s)');ylabel('V_x');xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
subplot(3,1,2)
plot(time(1:N-1),Vy0(1:N-1)/TT1*Vnorm,'k','Linewidth',2)
xlabel('time(s)');ylabel('V_y');xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
subplot(3,1,3)
plot(time(1:N-1),Vz0(1:N-1)/TT1*Vnorm,'k','Linewidth',2);
xlabel('time(s)');ylabel('V_z');xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
title([method ' ' 'Method']);
%%
figure(40)
subplot(2,2,1)
Vp0=sqrt(Vx0.^2+Vy0.^2+Vz0.^2)*Vnorm/TT1;
plot(time(1:N-1),Speedmax0*Vnorm*ones(1,N-1),'r','Linewidth',2.5);
hold on;
plot(time(1:N-1),Vp0(1:N-1),'--','Linewidth',1.5);
xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,max(Vp0)*1.2]);
xlabel('time(s)');ylabel('V(m/s)');grid on;
% legend('Vconst','Vcone');title([method ' ' 'Method']);
subplot(2,2,2)
plot(time(1:N-1),Speedmax1*Vnorm*ones(1,N-1),'r','Linewidth',2.5);
hold on;
Vp1=sqrt(Vx1.^2+Vy1.^2+Vz1.^2)*Vnorm/TT1;
plot(time(1:N-1),Vp1(1:N-1),'--','Linewidth',1.5);
xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,max(Vp1)*1.2]);
xlabel('time(s)');ylabel('V(m/s)');grid on;
% legend('Vconst','Vcone');title([method ' ' 'Method']);
subplot(2,2,3)
plot(time(1:N-1),Speedmax2*Vnorm*ones(1,N-1),'r','Linewidth',2.5);
hold on;
Vp2=sqrt(Vx2.^2+Vy2.^2+Vz2.^2)*Vnorm/TT1;
plot(time(1:N-1),Vp2(1:N-1),'--','Linewidth',1.5);
xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,max(Vp2)*1.2]);
xlabel('time(s)');ylabel('V(m/s)');grid on;
% legend('Vconst','Vcone');title([method ' ' 'Method']);
subplot(2,2,4)
plot(time(1:N-1),Speedmax3*Vnorm*ones(1,N-1),'r','Linewidth',2.5);
hold on;
Vp3=sqrt(Vx3.^2+Vy3.^2+Vz3.^2)*Vnorm/TT1;
plot(time(1:N-1),Vp3(1:N-1),'--','Linewidth',1.5);
xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,max(Vp3)*1.2]);
xlabel('time(s)');ylabel('V(m/s)');grid on;
% legend('Vconst','Vcone');title([method ' ' 'Method']);
%%
figure(400)
subplot(2,1,1)
plot(time(1:N-1),Vp0(1:N-1),'k:x','Markersize',5,'Linewidth',1);hold on;
plot(time(1:N-1),Vp1(1:N-1),'kd--','Markersize',3,'Linewidth',1);hold on;
plot(time(1:N-1),Vp2(1:N-1),'k:.','Markersize',8,'Linewidth',1);hold on;
plot(time(1:N-1),Vp3(1:N-1),'k-.o','Markersize',3,'Linewidth',1);hold on;
% xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,max(Vp0)*1.2]);
% legend('Missile-1','Missile-2','Missile-3','Missile-4','Location','NorthEast');
xlabel('time(s)');ylabel('Vel.(m/s)');grid on;
subplot(2,1,2)
plot(time((1:N-2)),sqrt((Ux0/TT2).^2+(Uy0/(TT2)).^2+(Uz0/(TT2)).^2)*anorm*m0/1000,'k:x','Markersize',5,'Linewidth',1);hold on;
plot(time((1:N-2)),sqrt((Ux1/TT2).^2+(Uy1/(TT2)).^2+(Uz1/(TT2)).^2)*anorm*m0/1000,'kd--','Markersize',3,'Linewidth',1);hold on;
plot(time((1:N-2)),sqrt((Ux2/TT2).^2+(Uy2/(TT2)).^2+(Uz2/(TT2)).^2)*anorm*m0/1000,'k:.','Markersize',8,'Linewidth',1);hold on;
plot(time((1:N-2)),sqrt((Ux3/TT2).^2+(Uy3/(TT2)).^2+(Uz3/(TT2)).^2)*anorm*m0/1000,'k-.o','Markersize',3,'Linewidth',1);hold on;
grid on;xlabel('time(s)');ylabel('Thrust(kN)');
xlim([0,tau*N*TT1*Tnorm*1.05]);ylim([0,Accel_max*m0/1000*1.2]);
legend('Missile-1','Missile-2','Missile-3','Missile-4','Location','NorthEast');
%%
phi0 = atan(Vx0./Vy0);
psi0 = asin(Vz0./sqrt(Vx0.^2+Vy0.^2+Vz0.^2));
phi1 = atan(Vx1./Vy1);
psi1 = asin(Vz1./sqrt(Vx1.^2+Vy1.^2+Vz1.^2));
phi2 = atan(Vx2./Vy2);
psi2 = asin(Vz2./sqrt(Vx2.^2+Vy2.^2+Vz2.^2));
phi3 = atan(Vx3./Vy3);
psi3 = asin(Vz3./sqrt(Vx3.^2+Vy3.^2+Vz3.^2));
figure(55)
plot(time((1:N-1)),phi0(1:N-1)*180/pi,'k:x','Markersize',5,'Linewidth',1);hold on;
plot(time((1:N-1)),phi1(1:N-1)*180/pi,'kd--','Markersize',3,'Linewidth',1);hold on;
plot(time((1:N-1)),phi2(1:N-1)*180/pi,'k:.','Markersize',8,'Linewidth',1);hold on;
plot(time((1:N-1)),phi3(1:N-1)*180/pi,'k-.o','Markersize',3,'Linewidth',1);hold on;
grid on;xlabel('time(s)');ylabel('\phi(deg)');
legend('Missile-1','Missile-2','Missile-3','Missile-4','Location','NorthEast');
figure(66)
plot(time((1:N-1)),psi0(1:N-1)*180/pi,'k:x','Markersize',5,'Linewidth',1);hold on;
plot(time((1:N-1)),psi1(1:N-1)*180/pi,'kd--','Markersize',3,'Linewidth',1);hold on;
plot(time((1:N-1)),psi2(1:N-1)*180/pi,'k:.','Markersize',8,'Linewidth',1);hold on;
plot(time((1:N-1)),psi3(1:N-1)*180/pi,'k-.o','Markersize',3,'Linewidth',1);hold on;
grid on;xlabel('time(s)');ylabel('\psi(deg)');
legend('Missile-1','Missile-2','Missile-3','Missile-4','Location','NorthEast');
%%
figure(5)
subplot(2,2,1);
plot(time((1:N-1)),phi0(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\phi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,2);
plot(time((1:N-1)),phi1(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\phi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,3);
plot(time((1:N-1)),phi2(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\phi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,4);
plot(time((1:N-1)),phi3(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\phi');xlim([0,tau*N*TT1*Tnorm*1.05]);
figure(6)
subplot(2,2,1);
plot(time((1:N-1)),psi0(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\psi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,2);
plot(time((1:N-1)),psi1(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\psi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,3);
plot(time((1:N-1)),psi2(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\psi');xlim([0,tau*N*TT1*Tnorm*1.05]);
subplot(2,2,4);
plot(time((1:N-1)),psi3(1:N-1)*180/pi,'k','Linewidth',2);
xlabel('time(s)');ylabel('\psi');xlim([0,tau*N*TT1*Tnorm*1.05]);
% %% %
% figure(7)
% subplot(2,1,1)
% plot(time(1:N)*obj,Gaps(1:N),'k','Linewidth',2)
% xlabel('time(s)');ylabel({'$gap_{s}$'},'Interpreter','latex');
% xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
% subplot(2,1,2)
% plot(time(1:N)*obj,Gapc(1:N),'k','Linewidth',2)
% xlabel('time(s)');ylabel({'$gap_{c}$'},'Interpreter','latex');
% xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
% %%
% figure(8)
% subplot(3,1,1);
% plot(time,Xk(end,:)*Lnorm,'k','Linewidth',1.5);
% xlabel('time(s)');ylabel({'$x$'},'Interpreter','latex');
% xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
% subplot(3,1,2);
% plot(time,Yk(end,:)*Lnorm,'k','Linewidth',1.5);
% xlabel('time(s)');ylabel({'$y$'},'Interpreter','latex');
% xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
% subplot(3,1,3);
% plot(time,Zk(end,:)*Lnorm,'k','Linewidth',1.5);
% xlabel('time(s)');ylabel({'$z$'},'Interpreter','latex');
% xlim([0,tau*N*TT1*Tnorm*1.05]);grid on;
%%

% figure(9)
% plot3(Vx/TT1*Vnorm,Vy/TT1*Vnorm,Vz/TT1*Vnorm,'k','Linewidth',2);
% xlabel({'$V_x$'},'Interpreter','latex');
% ylabel({'$V_y$'},'Interpreter','latex');
% zlabel({'$V_z$'},'Interpreter','latex');
% grid on;axis equal;
hold on
%%

an0 = (diff(psi0)/(TT1*Tnorm*tau)).*Vp0(1:N-2);
at0 = (diff(Vp0)/(TT1*Tnorm*tau));
an1 = (diff(psi1)/(TT1*Tnorm*tau)).*Vp1(1:N-2);
at1 = (diff(Vp1)/(TT1*Tnorm*tau));
an2 = (diff(psi2)/(TT1*Tnorm*tau)).*Vp2(1:N-2);
at2 = (diff(Vp2)/(TT1*Tnorm*tau));
an3 = (diff(psi3)/(TT1*Tnorm*tau)).*Vp3(1:N-2);
at3 = (diff(Vp3)/(TT1*Tnorm*tau));
figure(10)
subplot(2,2,1)
plot(time((1:N-2)),an0,'r','Linewidth',2);hold on
plot(time((1:N-2)),at0,'b','Linewidth',2);
subplot(2,2,2)
plot(time((1:N-2)),an1,'r','Linewidth',2);hold on
plot(time((1:N-2)),at1,'b','Linewidth',2);
subplot(2,2,3)
plot(time((1:N-2)),an2,'r','Linewidth',2);hold on
plot(time((1:N-2)),at2,'b','Linewidth',2);
subplot(2,2,4)
plot(time((1:N-2)),an3,'r','Linewidth',2);hold on
plot(time((1:N-2)),at3,'b','Linewidth',2);
% plot(time((1:N-2)),sqrt(an.^2+at.^2),'r','Linewidth',2);
hold on;
%%
figure(101)
plot(time(1:end-2),sqrt((Ux0/TT2*anorm-1).^2+(Uy0/TT2*anorm-1).^2+(Uz0/TT2*anorm-1).^2),'--b','Linewidth',2);
grid on;
xlabel('time(s)');ylabel({'$a(t)$'},'Interpreter','latex');
legend('Speed change rate');
figure(102)
subplot(2,2,1)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);hold on
plot(time(1:end-2),sqrt((Ux0/TT2).^2+(Uy0/(TT2)).^2+(Uz0/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
subplot(2,2,2)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);hold on
plot(time(1:end-2),sqrt((Ux1/TT2).^2+(Uy1/(TT2)).^2+(Uz1/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
subplot(2,2,3)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);hold on
plot(time(1:end-2),sqrt((Ux2/TT2).^2+(Uy2/(TT2)).^2+(Uz2/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
subplot(2,2,4)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);hold on
plot(time(1:end-2),sqrt((Ux3/TT2).^2+(Uy3/(TT2)).^2+(Uz3/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
legend('Max Limit','Control');

%%
figure(100)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);
hold on
plot(time(1:end-2),sqrt((Ux0/TT2).^2+(Uy0/(TT2)).^2+(Uz0/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
legend('Max Limit','Control','Location','SouthEast');
figure(101)
plot([time(1) ,time(end-2)],[Accel_max Accel_max],'-r','Linewidth',2);
hold on
plot(time(1:end-2),sqrt((Ux1/TT2).^2+(Uy1/(TT2)).^2+(Uz1/(TT2)).^2)*anorm,'--b','Linewidth',2);
ylim([0,Accel_max*1.2]);grid on;
legend('Max Limit','Control','Location','SouthEast');

%%
hold on
figure(111)
t1x = (0:0.01:sqrt(max(T2k))+0.01)*Tnorm;
t2y = t1x.^2;
plot(t1x,t2y,'-r','Linewidth',2);
xlabel({'$t_1 (s)$'},'Interpreter','latex');
ylabel({'$t_2 (s^2)$'},'Interpreter','latex');hold on;grid on;
plot(T1k(2:end)*Tnorm,T2k(2:end)*Tnorm^2,'.--b','Linewidth',2,'MarkerSize',20);
figure(112);
subplot(2,2,1)
semilogy(errorx0*Lnorm,'.-k','Linewidth',1.5,'MarkerSize',16);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations');ylabel({'$\delta_x (m)$'},'Interpreter','latex');
subplot(2,2,2);
semilogy(errory0*Lnorm,'.-k','Linewidth',1.5,'MarkerSize',16);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations');ylabel({'$\delta_y (m)$'},'Interpreter','latex');
subplot(2,2,3);
semilogy(errorz0*Lnorm,'.-k','Linewidth',1.5,'MarkerSize',16);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations');ylabel({'$\delta_z (m)$'},'Interpreter','latex');
subplot(2,2,4);
semilogy(errort*Tnorm,'.-k','Linewidth',1.5,'MarkerSize',16);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations');ylabel({'$\delta_t (t)$'},'Interpreter','latex');
%%
figure(1121)
subplot(2,1,1)
semilogy(max([errorx0;errory0;errorz0]),'o-k','Linewidth',1.5,'MarkerSize',6,...
    'MarkerEdgeColor',[1,0,0],'MarkerFaceColor',[1,0.07,0.65]);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations','FontSize',15);
ylabel({'max $\delta_r $(m)'},'Interpreter','latex','FontSize',15);
subplot(2,1,2)
semilogy(errort*Tnorm,'o-k','Linewidth',1.5,'MarkerSize',6,...
    'MarkerEdgeColor',[1,0,0],'MarkerFaceColor',[1,0.07,0.65]);
grid on;xlim([0 s]);set(gca,'XTick',0:1:s);
xlabel('Iterations','FontSize',15);
ylabel({'$\delta_t $(s)'},'Interpreter','latex','FontSize',15);
% ylabel({'$t_s-t_f^2$'},'Interpreter','latex','FontSize',15);
%%
gamma = zeros(N-2,1);
for i = 1 :N-2
    U= [Ux0(i),Uy0(i),Uz0(i)];
    V = [Vx0(i),Vy0(i),Vz0(i)];
gamma(i) = acos(dot(U,V)/(norm(U)*norm(V)));
end
figure(324)
plot(time(1:end-2),gamma*180/pi,'--b','Linewidth',2);
