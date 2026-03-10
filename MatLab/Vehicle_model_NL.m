function [out] = Vehicle_model_NL(in)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

u = in(1);
v = in(2);
r = in(3);
nu = in(4);
epsilon = in(5);
omega11 = in(6);
omega12 = in(7);
omega21 = in(8);
omega22 = in(9);
% Coppia motrice [Nm]
Tdrive = in(10);
% Coppia frenante [Nm]
Tbrake = in(11);
% Angolo di sterzo [rad]
deltav = in(12);


% ModelloVeicolo_2025/2026

% 2) Definizione dei limiti sugli stati

u_lim = 1/u*[0 vp.Vmax]; % Questo è abbastanza chiaro, in caso va solo cambiata la scalatura
v_lim = 1/v*[-10 10]; % Questo si cerca di tenerlo attorno a 1
r_lim = 1/r*[-10 10]; % Qui mettiamoci un pi/2 
nu_lim = 1/nu*[-0.875 0.875]; %Per skid
%nu_lim = 1/nu*[-1 1]; % Questo in teoria permette di specificare la largezza del circuito, assunta costante. Dovrebbe esserci il modo di mettere una distanza variabile ma è follia, messo a 1
epsilon_lim = 1/epsilon*[-pi/4 pi/4]; % Un po' strano ma in realtà è solo pi/4
% Le velocità angolari abbastanza semplici, fermo al minimo, Vmax/u al massimo
omega11_lim = 1/omega11*[0 vp.Vmax/vp.Rr];
omega12_lim = 1/omega12*[0 vp.Vmax/vp.Rr];
omega21_lim = 1/omega21*[0 vp.Vmax/vp.Rr];
omega22_lim = 1/omega22*[0 vp.Vmax/vp.Rr];

% 3) Raggruppamento

% Vettore degli stati 
X = [u; v; r; nu; epsilon; omega11; omega12; omega21; omega22];
% Limiti
X_lim = [u_lim;v_lim;r_lim;nu_lim;epsilon_lim;omega11_lim;omega12_lim;omega21_lim;omega22_lim];
X_min = X_lim(:,1);
X_max = X_lim(:,2);

% 2) Definizione dei limiti sui controlli + Frenata rigenerativa

TdriveR_lim = 1/TdriveR*[-vp.Tregenmax vp.Tdrivemax];
TdriveL_lim = 1/TdriveL*[-vp.Tregenmax vp.Tdrivemax];
Tbrake_lim = 1/Tbrake*[-vp.Tbrakemax 0];
deltav_lim = 1/deltav*[-pi/2 pi/2];

% Differernziale similmeccanico aperto ***********
TdriveR = (Tdrive)/2; 
TdriveL = (Tdrive)/2;

% 2) Definizione dei limiti sui controlli + Frenata rigenerativa

TdriveR_lim = 1/TdriveR*[-vp.Tregenmax vp.Tdrivemax];
TdriveL_lim = 1/TdriveL*[-vp.Tregenmax vp.Tdrivemax];
Tbrake_lim = 1/Tbrake*[-vp.Tbrakemax 0];
deltav_lim = 1/deltav*[-pi/2 pi/2];

% 3) Raggruppamento

% Vettore dei controlli
U = [Tdrive; Tbrake; deltav];
% Limiti
U_lim = [TdriveR_lim;TdriveL_lim;Tbrake_lim;deltav_lim];
U_min = U_lim(:,1);
U_max = U_lim(:,2);

% Vincolo ulteriore sulla derivata dei controlli, sfrutta quello definito prima
%duk_ub = duk_ub./U;
%duk_lb = duk_lb./U;
%per ora sono come testo perchè non so quanto valgono queste derivate

% VARIABILI AUSILIARIE

% 1) Definizione simbolica deille variabili ausiliarie

% Trasferimento di carico longitudinale [N]
deltaZ = vp.m*vp.g*vp.h/vp.l;

% Trasferimento di carico laterale anteriore [N]
deltaZ1 = (((vp.kphi1/vp.kphi)*(vp.m*vp.g)*(vp.h-vp.qb)+(vp.m*vp.g*vp.a2/vp.l)*vp.q1+(vp.kphi1*vp.kphi2/vp.kphi)*((vp.m*vp.g*vp.a1/vp.l)*vp.q2/vp.kphi2p-(vp.m*vp.g*vp.a2/vp.l)*vp.q1/vp.kphi1p))*(1/vp.t1));

% Trasferimento di carico laterale posteriore [N]
deltaZ2 = (((vp.kphi1/vp.kphi)*(vp.m*vp.g)*(vp.h-vp.qb)+(vp.m*vp.g*vp.a1/vp.l)*vp.q1+(vp.kphi1*vp.kphi2/vp.kphi)*((vp.m*vp.g*vp.a2/vp.l)*vp.q2/vp.kphi2p-(vp.m*vp.g*vp.a1/vp.l)*vp.q1/vp.kphi1p))*(1/vp.t1));

% 2) Definizione dei limiti sulle variabili ausiliarie

deltaZ_lim = 1/deltaZ*[-2*vp.m*vp.g 2*vp.m*vp.g];
deltaZ1_lim = 1/deltaZ1*[-2*vp.m*vp.g 2*vp.m*vp.g];
deltaZ2_lim = 1/deltaZ2*[-2*vp.m*vp.g 2*vp.m*vp.g];

% 3) Raggruppamento

% Vettore delle variabili ausiliarie 
Zaus = [deltaZ;deltaZ1;deltaZ2];
% Limiti
Zaus_lim = [deltaZ_lim;deltaZ1_lim;deltaZ2_lim];
Zaus_min = Zaus_lim(:,1);
Zaus_max = Zaus_lim(:,2);

% EQUAZIONI 

% 1) Legge di sterzo

delta11 = -vp.delta10+vp.tau1*deltav+vp.eps1*(vp.t1/(2*vp.l))*(vp.t1*deltav)^2;
delta12 = vp.delta10+vp.tau1*deltav-vp.eps1*(vp.t1/(2*vp.l))*(vp.t1*deltav)^2;
delta21 = -vp.delta20+vp.tau2*deltav+vp.eps2*(vp.t2/(2*vp.l))*(vp.t2*deltav)^2;
delta22 = vp.delta20+vp.tau2*deltav-vp.eps2*(vp.t2/(2*vp.l))*(vp.t2*deltav)^2;

% 2) Velocità pneumatici sistema "Body

% FRONT LEFT
Vb11x = u-(r*vp.t1)/2; Vb11y = v+r*vp.a1;
% FRONT RIGHT
Vb12x = u+(r*vp.t1)/2; Vb12y = v+r*vp.a1;
% REAR LEFT
Vb21x = u-(r*vp.t2)/2; Vb21y = v-r*vp.a2;
% REAR RIGHT
Vb22x = u+(r*vp.t2)/2; Vb22y = v-r*vp.a2;

% 3) Angoli di assetto

% FRONT LEFT
beta11 = (v+r*vp.a1)/(u-r*vp.t1/2);
% FRONT RIGHT
beta12 = (v+r*vp.a1)/(u+r*vp.t1/2);
% REAR LEFT
beta21 = (v-r*vp.a2)/(u-r*vp.t2/2);
% REAR RIGHT
beta22 = (v-r*vp.a2)/(u+r*vp.t2/2);

% 3b) Velocità sistema "Wheel"

% FRONT LEFT
Vw11x = [cos(delta11) sin(delta11)]*[Vb11x;Vb11y];
Vw11y = [-sin(delta11) cos(delta11)]*[Vb11x;Vb11y];
% FRONT RIGHT
Vw12x = [cos(delta12) sin(delta12)]*[Vb12x;Vb12y];
Vw12y = [-sin(delta12) cos(delta12)]*[Vb12x;Vb12y];
% REAR LEFT
Vw21x = [cos(delta21) sin(delta21)]*[Vb21x;Vb21y];
Vw21y = [-sin(delta21) cos(delta21)]*[Vb21x;Vb21y];
% REAR RIGHT
Vw22x = [cos(delta22) sin(delta22)]*[Vb22x;Vb22y];
Vw22y = [-sin(delta22) cos(delta22)]*[Vb22x;Vb22y];

% 4) Angoli di deriva

% FRONT LEFT
alpha11 = delta11-beta11;
% FRONT RIGHT
alpha12 = delta12-beta12;
% REAR LEFT
alpha21 = delta21-beta21;
% REAR RIGHT
alpha22 = delta22-beta22;

% 5) Scorrimenti teorici

% Longitudinali
sigmax11 = (((u-r*vp.t1/2)*cos(delta11)+(v+r*vp.a1)*sin(delta11))-omega11*vp.Rr)/(omega11*vp.Rr);
sigmax12 = (((u+r*vp.t1/2)*cos(delta12)+(v+r*vp.a1)*sin(delta12))-omega12*vp.Rr)/(omega12*vp.Rr);
sigmax21 = (((u-r*vp.t2/2)*cos(delta21)+(v-r*vp.a2)*sin(delta21))-omega21*vp.Rr)/(omega21*vp.Rr);
sigmax22 = (((u+r*vp.t2/2)*cos(delta22)+(v-r*vp.a2)*sin(delta22))-omega22*vp.Rr)/(omega22*vp.Rr);
% Laterali
sigmay11 = ((v+r*vp.a1)*cos(delta11)-(u-r*vp.t1/2)*sin(delta11))/(omega11*vp.Rr);
sigmay12 = ((v+r*vp.a1)*cos(delta12)-(u+r*vp.t1/2)*sin(delta12))/(omega12*vp.Rr);
sigmay21 = ((v-r*vp.a2)*cos(delta21)-(u-r*vp.t2/2)*sin(delta21))/(omega21*vp.Rr);
sigmay22 = ((v-r*vp.a2)*cos(delta22)-(u+r*vp.t2/2)*sin(delta22))/(omega22*vp.Rr);

% 6) Scorrimenti pratici

% SR = slip ratio (quelli lungo x)
% SA = slip angle (quelli lungo y)
SR11 = -(sigmax11/(1+sigmax11));
SR12 = -(sigmax12/(1+sigmax12));
SR21 = -(sigmax21/(1+sigmax21));
SR22 = -(sigmax22/(1+sigmax22));
SA11 = -(sigmay11/(1+sigmax11));
SA12 = -(sigmay12/(1+sigmax12));
SA21 = -(sigmay21/(1+sigmax21));
SA22 = -(sigmay22/(1+sigmax22));

% 7) Aerodinamica

% Resistenza aerodinamica [N]
Xa = 0.5*vp.rho*vp.S*vp.Cx*u^2;

% Deportanza anteriore e posteriore [N]
Z1a = 0.5*vp.rho*vp.S*vp.Cz1*u^2;
Z2a = 0.5*vp.rho*vp.S*vp.Cz2*u^2;

% 8) Carichi sulle ruote

Z10 = vp.m*vp.g*vp.a2/vp.l;
Z20 = vp.m*vp.g*vp.a1/vp.l;

Fz11 = 0.5*(Z10+Z1a-deltaZ)-deltaZ1;
Fz12 = 0.5*(Z10+Z1a-deltaZ)+deltaZ1;
Fz21 = 0.5*(Z20+Z2a+deltaZ)-deltaZ2;
Fz22 = 0.5*(Z20+Z2a+deltaZ)+deltaZ2;

% Derivate
dot_u = (((X1+X2-Xa)/vp.m)+v*r);
dot_v = (((Y1+Y2)/vp.m)-u*r);
dot_r = ((Y1*vp.a1-Y2*vp.a2+deltaX1*vp.t1+deltaX2*vp.t2)/vp.Jz);
dot_nu = (u*sin(epsilon)+v*cos(epsilon))*sf;
dot_epsilon = sf*r;
dot_omega11 = ((T11-FX11*vp.Rr)/vp.tyre.IYY)*sf;
dot_omega12 = ((T12-FX12*vp.Rr)/vp.tyre.IYY)*sf;
dot_omega21 = ((T21-FX21*vp.Rr)/vp.tyre.IYY)*sf;
dot_omega22 = ((T22-FX22*vp.Rr)/vp.tyre.IYY)*sf;
% Scalatura
dot_x = [dot_u; dot_v; dot_r; dot_nu; dot_epsilon; 
            dot_omega11; dot_omega12; dot_omega21; dot_omega22];

out = dot_x;

end