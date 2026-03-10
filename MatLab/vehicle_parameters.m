% DATI VEICOLO 

% % parametriVeicolo_OCP_FSAE.m - Parametri del veicolo (ET-16) DSS FSG24 +
% % CFD 2024 
% % Crea una struct che si pappa tutti i parametri che verranno usati dal
% % modello veicolo
% import casadi.*
% %% Parametri generici
vp.g = 9.81; % [m/s^2], accelerazione di gravità
%vp.ms = 271; %[kg]
%vp.mn1 = 20;
%vp.mn2 = 20;
vp.m1=135;  % [kg], massa anteriore
vp.m2=165;  % [kg], massa anteriore
vp.m = vp.m1+vp.m2; % [kg], massa veicolo
vp.t1 = 1.252; % [m], carreggiata anteriore
vp.t2 = 1.136; % [m], carreggiata posteriore
vp.l = 1.527; % [m], passo
vp.WB = 0.45; % weight balance 
vp.a1 = vp.l*(1-vp.WB); % [m], semipasso anteriore
vp.a2 = vp.l*vp.WB; % [m], semipasso posteriore
vp.h = 0.29; % [m], altezza CoG
vp.h1 = 0.24461; % [m], altezza CoG massa non sospesa anteriore
vp.h2 = 0.24461; % [m], altezza CoG massa non sospesa posteriore
vp.q1 = 0.0354356; % [m], altezza NRC anteriore
vp.q2 = 0.0730342; % [m], altezza NRC posteriore
vp.qb = (vp.a2*vp.q1+vp.a1*vp.q2)/vp.l; % [m], altezza del NRC globale intercettata sull'asse di non rollio dalla verticale al CoG
vp.Rr = 1.505/(2*pi); % [m], raggio di rotolamento pneumatico
vp.f = 0.015; % [], coefficiente di attrito a rotolamente pneumatico
% f e Rr si dovrebbero calcolare dalla magic formula ma diventa un po'
% complicato il tutto facendo in quel modo
vp.rho = 1.225; % [kg/m^3], densità aria
vp.S = 1.139; % [m^2], superficie anteriore 
vp.Cz = 3.328; % [], Cz complessivo
vp.AB = 0.485; % [], aero-balance
vp.Cz1 = vp.Cz*vp.AB; % [], coeficiente deportanza anteriore
vp.Cz2 = vp.Cz*(1-vp.AB); % [], coefficiente deportanza posteriore
vp.Cx = 1.451; % [], coefficiente resistenza aerodinamica
vp.Jx = (vp.m/12)*(vp.t1^2+vp.h^2); % [kgm^2], sono una cazzata al 100% perché sono i momenti d'inerzia di un parallelepipedo ma non è che si può fare molto altro
vp.Jy = (vp.m/12)*(vp.l^2+vp.h^2); % [kgm^2], idem
vp.Jz = (vp.m/12)*(vp.l^2+vp.t1^2); % [kgm^2], idem
vp.Jxy = 0; % [kgm^2], idem
vp.Jxz = 0; % [kgm^2], idem, questo era indicato ma sono sicuro sia falso
vp.Jyz = 0; % [kgm^2], idem
%% Camber e pressione
vp.gamma11 = deg2rad(-1.5);
vp.gamma12 = deg2rad(-1.5);
vp.gamma21 = deg2rad(-1);
vp.gamma22 = deg2rad(-1);
% [deg], segno angoli di camber secondo convenzione, guardando la ruota da
% davanti:
% Camber negativo = \___/
% Camber positivo = /___\
vp.press11 = 100000; % [Pa], pressione di gonfiaggio anteriore sinistra
vp.press12 = 100000; % [Pa], pressione di gonfiaggio anteriore destra
vp.press21 = 100000; % [Pa], pressione di gonfiaggio posteriore sinistra
vp.press22 = 100000; % [Pa], pressione di gonfiaggio posteriore destra

%% Punti sospensione nuovi
%% FRONT LEFT (11)
vp.sosp.FLq = [1528.87; 626.3; 40.8]; % [mm], centro ruota (cerchione)
vp.sosp.FLs = [1528.87; 626.3; -194.2]; % [mm], centro impronta (pneumatico)
% Attacchi braccetti sospensioni (tutto in [mm])
vp.sosp.FLn1 = [1610.68; 252; -80.15];
vp.sosp.FLn2 = [1420; 265.37; -71.18];
vp.sosp.FLh1 = [1630.46; 262.5; 84.73];
vp.sosp.FLh2 = [1414.75; 263.5; 83.04];
vp.sosp.FLe = [1511; 550.9; 160.9];
vp.sosp.FLk = [1534; 570; -56.2];
% Attacchi tirante sterzo (tutto in [mm])
vp.sosp.FLp = [1498.5; 243.63; -51.86];
vp.sosp.FLi = [1474; 535; -15.2];
% Attacchi push rod (tutto in [mm])
vp.sosp.FLc = [1509.8; 295; 397.8];
vp.sosp.FLd = [1510.648; 523.987; 176.571];
% Attacchi rocker (tutto in [mm])
vp.sosp.FLg = [1509.8; 234.000; 349.8];
vp.sosp.FLb = [1509.8; 230.402; 427.770];
vp.sosp.FLf = [1509.8; 225.77; 379.8];
% Settaggi barra antirollio (tutto in [mm]) -> cambiati, sere controllare
% quelli nuovi
% vp.sosp.FLLL = [1574 1560 1546 1534 1522 % x
%   267 267 267 267 267 % y
%   -105 -105 -105 -105 -105]; % z
vp.sosp.FLl = [1536.864; 233.017; 379.8]; % tirante di barra
% vp.sosp.FLl = vp.sosp.FLLL(:,5); % Punto attuale barra
vp.sosp.FLm = [1610.393; 37.910; 379.790];
vp.sosp.FLr = [1509.8; 0; 379.790];
% Attacco molla (tutto in [mm])
vp.sosp.FLa = [1509.8; 54.5; 374.79];
%% FRONT RIGHT (12)
% A regola basta una specchiatura rispetto al piano xz quindi na roba tipo
% R = [1 0 0; 0 -1 0; 0 0 1] per quello che si è scritto
RsymFRONT = [1 0 0; 0 -1 0; 0 0 1]; % Matrice di specchiatura anteriore
vp.sosp.FRq = RsymFRONT*vp.sosp.FLq; % [mm], centro ruota (cerchione)
vp.sosp.FRs = RsymFRONT*vp.sosp.FLs; % [mm], centro impronta (pneumatico)
% Attacchi braccetti sospensioni (tutto in [mm])
vp.sosp.FRn1 = RsymFRONT*vp.sosp.FLn1;
vp.sosp.FRn2 = RsymFRONT*vp.sosp.FLn2;
vp.sosp.FRh1 = RsymFRONT*vp.sosp.FLh1;
vp.sosp.FRh2 = RsymFRONT*vp.sosp.FLh2;
vp.sosp.FRe = RsymFRONT*vp.sosp.FLe;
vp.sosp.FRk = RsymFRONT*vp.sosp.FLk;
% Attacchi tirante sterzo (tutto in [mm])
vp.sosp.FRp = RsymFRONT*vp.sosp.FLp;
vp.sosp.FRi = RsymFRONT*vp.sosp.FLi;
% Attacchi push rod (tutto in [mm])
vp.sosp.FRc = RsymFRONT*vp.sosp.FLc;
vp.sosp.FRd = RsymFRONT*vp.sosp.FLd;
% Attacchi rocker (tutto in [mm])
vp.sosp.FRg = RsymFRONT*vp.sosp.FLg;
vp.sosp.FRb = RsymFRONT*vp.sosp.FLb;
vp.sosp.FRf = RsymFRONT*vp.sosp.FLf;
% Settaggi barra antirollio (tutto in [mm])
% vp.sosp.FRl = [1482.736; 233.017; 379.8]; % tirante di barra
% vp.sosp.FRm = [1409.207; 37.910; 379.790];
% vp.sosp.FRr = [1509.8; 0; 379.790];
vp.sosp.FRl = RsymFRONT*vp.sosp.FLl;
vp.sosp.FRm = RsymFRONT*vp.sosp.FLm;
vp.sosp.FRr = RsymFRONT*vp.sosp.FLr;
%vp.sosp.FRLL = RsymFRONT*vp.sosp.FLLL;
%vp.sosp.FRl = vp.sosp.FRLL(:,5);
%vp.sosp.FRm = RsymFRONT*vp.sosp.FLm;
%vp.sosp.FRr = RsymFRONT*vp.sosp.FLr;
% Attacco molla (tutto in [mm])
vp.sosp.FRa = RsymFRONT*vp.sosp.FLa;
%% REAR LEFT
% Qui non si può specchiare nada
vp.sosp.RLq = [0; 568; 40.8]; % [mm], centro ruota (cerchione)
vp.sosp.RLs = [0; 568; -194.2]; % [mm], centro impronta (pneumatico)
% Attacchi braccetti sospensioni (tutto in [mm])
vp.sosp.RLn1 = [155.97; 290; -81.85];
vp.sosp.RLn2 = [-163.76; 295; -90.33];
vp.sosp.RLh1 = [163.92; 297.7; 37.9];
vp.sosp.RLh2 = [-154.392; 297.7; 30.037];
vp.sosp.RLe = [59.22; 500.75; 118.97];
vp.sosp.RLk = [19.95; 523.55; -68.7];
% Attacchi tirante convergenza (tutto in [mm])
vp.sosp.RLp = [-90.86; 295; -21.2];
vp.sosp.RLi = [-94.91; 543.5; 44.37];
% Attacchi push rod (tutto in [mm])
vp.sosp.RLc = [50; 322; 338.3];
vp.sosp.RLd = [49.77; 470.19; 131.75];
% Attacchi rocker (tutto in [mm])
vp.sosp.RLg = [50; 246; 296.8];
vp.sosp.RLb = [50; 242.15; 379.25];
vp.sosp.RLf = [50; 237.265; 328.8];
% Settaggi barra antirollio (tutto in [mm])
% vp.sosp.RLLL = [13.71 33.71 53.71 73.71 93.71 % x
    %245.29 245.29 245.29 245.29 245.29 % y
    %-120 -120 -120 -120 -120]; % z
vp.sosp.RLl = [77.06; 244.52; 328.8];
% vp.sosp.RLl = vp.sosp.RLLL(:,3); % In realtà la barra dietro non ci sarebbe ma per evitare che appaia basta azzerare il suo contributo alle equazioni
vp.sosp.RLm = [156.59; 41.82; 328.61];
vp.sosp.RLr = [50; 0; 328.61];
% Attacco molla (tutto in [mm])
vp.sosp.RLa = [50; 65; 330.61];
%% REAR RIGHT
% Qui specchiatura sì
RsymREAR = [1 0 0; 0 -1 0; 0 0 1];
vp.sosp.RRq = RsymREAR*vp.sosp.RLq; % [mm], centro ruota (cerchione)
vp.sosp.RRs = RsymREAR*vp.sosp.RLs; % [mm], centro impronta (pneumatico)
% Attacchi braccetti sospensioni (tutto in [mm])
vp.sosp.RRn1 = RsymREAR*vp.sosp.RLn1;
vp.sosp.RRn2 = RsymREAR*vp.sosp.RLn2;
vp.sosp.RRh1 = RsymREAR*vp.sosp.RLh1;
vp.sosp.RRh2 = RsymREAR*vp.sosp.RLh2;
vp.sosp.RRe = RsymREAR*vp.sosp.RLe;
vp.sosp.RRk = RsymREAR*vp.sosp.RLk;
% Attacchi tirante sterzo (tutto in [mm])
vp.sosp.RRp = RsymREAR*vp.sosp.RLp;
vp.sosp.RRi = RsymREAR*vp.sosp.RLi;
% Attacchi push rod (tutto in [mm])
vp.sosp.RRc = RsymREAR*vp.sosp.RLc;
vp.sosp.RRd = RsymREAR*vp.sosp.RLd;
% Attacchi rocker (tutto in [mm])
vp.sosp.RRg = RsymREAR*vp.sosp.RLg;
vp.sosp.RRb = RsymREAR*vp.sosp.RLb;
vp.sosp.RRf = RsymREAR*vp.sosp.RLf;
% Settaggi barra antirollio (tutto in [mm])
% vp.sosp.RRl = [22.94; -244.52; 328.8];
% vp.sosp.RRm = [-56.59; -41.82; 328.61];
vp.sosp.RRl = RsymREAR*vp.sosp.RLl;
vp.sosp.RRm = RsymREAR*vp.sosp.RLm;
vp.sosp.RRr = RsymREAR*vp.sosp.RLr;
% vp.sosp.RRLL = RsymREAR*vp.sosp.RLLL;
% vp.sosp.RRl = vp.sosp.RRLL(:,3);
% vp.sosp.RRm = RsymREAR*vp.sosp.RLm;
% vp.sosp.RRr = RsymREAR*vp.sosp.RLr;
% Attacco molla (tutto in [mm])
vp.sosp.RRa = RsymREAR*vp.sosp.RLa;
%% Elastocinematica (me cojo...)
vp.k1 = 41.15; % [N/mm], rigidezza molle anteriore
vp.k2 = 61.3; % [N/mm]. rigidezza molle posteriore
vp.p = 110.07; % [N/mm], rigidezza pneumatici in condizioni standard (da file .tir)
vp.E = 115e3; % [N/mm^2 o MPa], modulo di Young titanio della barra
vp.nu = 0.3; % [], modulo di Poisson, idem come sopra (penso?)

vp.sosp.FRLL = RsymFRONT * vp.sosp.FLl; % [mm], attacco barra antirollio anteriore
vp.sosp.RRLL = RsymREAR * vp.sosp.RLl; % [mm], attacco barra antirollio posteriore
vp.L1 = norm(vp.sosp.FLm-vp.sosp.FLr)+norm(vp.sosp.FRm-vp.sosp.FRr); % [mm], questo calcola la lunghezza della barra da punto di attacco al coltello a punto medio, a sinistra e a destra con un bel più
vp.L2 = norm(vp.sosp.RLm-vp.sosp.RLr)+norm(vp.sosp.RRm-vp.sosp.RRr); % [mm], uguale ma per barra al posteriore

% vp.raggioext1 = 6.35; % [mm], raggio esterno barra anteriore
% vp.spessore1 = 2.41; % [mm], spessore barra anteriore
% vp.raggioext2 = 5; % [mm] raggio esterno barra posteriore
% vp.spessore2 = 2; % [mm], spessore barra posteriore

% OCIO! Questi sono per la condizione a spostamento nullo, QUESTI DIPENDONO
% DAL MOTO DELLA SOSPENSIONE, NON SONO VALORI FISSI!
vp.IB1 = 0.764; % [], rapporto di installazione della molla all'anteriore
vp.IB2 = 0.795; % [], rapporto di installazione della molla al posteriore
vp.IM1 = 0.3; % [], rapporto di installazione della barra all'anteriore
vp.IM2 = 0.3; % [], rapporto di installazione della barra al posteriore
vp.b1 = norm(vp.sosp.FLm-vp.sosp.FLl); % [mm], lunghezza coltello effettivamente interessata dall'azione della barra
vp.b2 = norm(vp.sosp.RLm-vp.sosp.RLl); % [mm], idem ma al posteriore
% Conti time

% provo a inserire lo script per le nuove barre e vedo se funziona
% FRONT
Wvf = [4.5 6]; % Spessori barra -> si può scegliere tra due barre di spessori diversi
Thvf = linspace(0, pi*13/45, 4); % Angoli di settaggio della barra
vp.Lf = 100; % Lunghezza del tratto utile di barra
vp.h0f = 24; % Altezza iniziale del coltello
vp.h1f = 7; % Altezza finale del coltello

%REAR
Wvr = [6 6.6]; % Spessori barra -> si può scegliere tra due barre di spessori diversi
Thvr = linspace(0, pi*13/45, 4); % Angoli di settaggio della barra
vp.Lr = 105; % Lunghezza del tratto utile di barra
vp.h0r = 24; % Altezza iniziale del coltello
vp.h1r = 12; % Altezza finale del coltello

vp.kb1 = ARB(vp.E, Wvf, Thvf, vp.Lf, vp.h0f, vp.h1f); % [Nmm]
vp.kb2 = ARB(vp.E, Wvr, Thvr, vp.Lr, vp.h0r, vp.h1r); % [Nmm]

% vp.kb1 = Kb(vp.E,vp.nu,vp.raggioext1,vp.spessore1,vp.L1); % [Nmm] a regola
% vp.kb2 = Kb(vp.E,vp.nu,vp.raggioext2,vp.spessore2,vp.L2); % [Nmm] a regola
vp.kphi1s = ((vp.k1*(vp.IM1^2)+vp.kb1*(2/(vp.b1^2))*(vp.IB1^2))*(vp.t1^2)/2)*1e3; % [Nm] a regola
vp.kphi2s = ((vp.k2*(vp.IM2^2)+vp.kb2*(2/(vp.b2^2))*(vp.IB2^2))*(vp.t2^2)/2)*1e3; % [Nm] a regola
vp.kphi1p = ((vp.p*(vp.t1^2))/2)*1e3; % [Nm] a regola
vp.kphi2p = ((vp.p*(vp.t2^2))/2)*1e3; % [Nm] a regola
vp.kphi1 = (vp.kphi1s*vp.kphi1p)/(vp.kphi1s+vp.kphi1p); % [Nm] a regola
vp.kphi2 = (vp.kphi2s*vp.kphi2p)/(vp.kphi2s+vp.kphi2p); % [Nm] a regola
vp.kphi = vp.kphi1+vp.kphi2; % [Nm] a regola
% Tutti quelli qui su in realtà hanno anche radianti mi pare al
% denominatore, quindi per passarli in qualcosa di utilizzabile quando si
% parla ricordarsi di fare un bel *pi/180 per riottenere Nm/deg che
% dovrebbe essere l'unità più accettabile
% %% Coefficienti di sterzo
% % Anteriore
vp.tau1 = 2*0.213632; % Rapporto di sterzatura parallelo anteriore
vp.eps1 = 0.40; % Coefficiente di Ackermann anteriore (in % 75% o 40%)
% vp.eps1 = 0.005; % Coefficiente di Ackermann anteriore
% vp.c11 = 0.0009;
% Posteriore
vp.tau2 = 0; % Rapporto di sterzatura parallelo posteriore
vp.eps2 = 0; % Coefficiente di Ackermann posteriore
% vp.c22 = 0;
% Convergenza
vp.delta10 = deg2rad(0); % Anteriore
vp.delta20 = deg2rad(0); % Posteriore
% [deg], convergenza con segno positivo, divergenza segno negativo

%% Pneumatico
%**************************************************************************
% ATTENZIONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
%**************************************************************************
% PURE CHE NON HAI LETTO ALCUN ALTRO COMMENTO LEGGI ALMENO MEEEEEEEEEEEEEEE
%**************************************************************************
% CON QUESTO CODICE CI PUOI FARE IL CAZZO CHE TI PARE, PURE USARTELO PER 
% GLI ESAMI
%**************************************************************************
% MA
%**************************************************************************
% OCCHIO A CHE PNEUMATICO STA VENENDO USATO! GLI PNEUMATICI ATTUALI PIRELLI
% NON SONO STATI FITTATI DA NOI E IN TEORIA CI STA UN NDA PER POTERLI USARE
%**************************************************************************
% QUINDI FINO A QUANDO RESTANO INTERNI ALLA SQUADRA CORSE TUTTO OK, FUORI
% NON USARLI PLZ!
% SE VUOI USARE IL CODICE FUORI CAMBIA PNEUMATICO! CI STA UN FILE MAT
% MF61FittedTyres_Final.mat <---------------------------------------------
% USA QUALUNQUE PNEUMATICO NEL FILE O QUALUNQUE ALTRO TIR VUOI CON MF61 
% COME FIT MA NON USARE I PIRELLI NEL MONDO VERO!
%**************************************************************************
% Gli pneumatici vanno scritti, al momento, in maniera hardcore:
% Si prende il file tir o si fa leggere il file mat, si copincolla ogni
% cosa e ci si aggiunge un bel vp. all'inizio di ogni costante. Se si vuole
% cambiare pneumatico visto che è un lavoro lungo e fastidioso fare il
% cambio ogni volta è consigliabile lasciare gli pneumatici scritti,
% commentarli e mettere i nuovi nello stesso formato SPECIFICANDO di che
% pneumatico si tratta con il commento "%%" O SI AMMATTISCE POI
% NON TUTTA LA ROBA CHE CI STA IN UN FILE TIR È UTILE O CE NE FREGA
% MINIMAMENTE QUALCOSA! È COMUNQUE CONSIGLIABILE APPICCICARCI TUTTO QUELLO
% CHE CI STA ALL'INTERNO COMUNQUE!
% NOTA ESTREMAMENTE DOLENTE: I FATTORI LAMBDA (LMUX E LMUY IN PARTICOLARE)
% NON TUTTI I FILE LI CONTENGONO E A SECONDA DELLE CONDIZIONI IN CUI SONO
% STATE FATTE LE PROVE SONO COME GIORNO E NOTTE PER QUELLO CHE VIENE FUORI!
% COME NOTA GENERALE: I PIRELLI VANNO BENE COME SONO SENZA FATTORI LAMBDA 
% (O MEGLIO, FATTORI LAMBDA UNITARI), GLI PNEUMATICI FITTATI DA NOI IN QUEL
% FAMOSO FILE HANNO **TUTTI** BISOGNO CHE LMUX E LMUY SIANO 2/3 O SUCCEDONO
% COSE POCO SIMPATICHE COME VALORI DI ADERENZA DI 3 E ROTTI, CAUSA PROVE 
% EFFETTUATE SU CARTA VETRATA O QUASI
% IL PROBLEMA NON È RISOLVIBILE FACENDO SOLO 2/3 QUANDO SI VA A VEDERE
% L'ADERENZA IN QUANTO ANCHE LE FORZE E I MOMENTI DIPENDONO DA QUESTA!
% QUINDI SPECIFICA I FATTORI LAMBDA SEMPRE QUI DENTRO SE SERVONO!
%**************************************************************************
% TUTTI I VALORI NEI FILE TIR SONO SPECIFICATI IN SISTEMA SI (kg,m,rad,s,etc)
%**************************************************************************
%% 185x40 13S 9Z0481 FSAE Slick Pirelli
%----------------------------------------------------------------model
% [MODEL]
% PROPERTY_FILE_FORMAT     = 'MF-TYRE'
% TYPE                     = 'CAR'
% FITTYP                   = 61                  $Magic Formula Version number
% TYRESIDE                 = 'Right'             
vp.tyre.LONGVL                   = 27.7778             ; % Nominal speed
vp.tyre.VXLOW                    = 1                   ; % Lower boundary of slip calculation
%-----------------------------------------------------------dimensions
% [DIMENSION]
vp.tyre.UNLOADED_RADIUS          = 0.2391              ; % Free tyre radius
vp.tyre.WIDTH                    = 0.185               ; % Nominal section width of the tyre
vp.tyre.ASPECT_RATIO             = 0.4                 ; % Nominal aspect ratio
vp.tyre.RIM_RADIUS               = 0.1651              ; % Nominal rim radius
vp.tyre.RIM_WIDTH                = 0.1524              ; % Rim width
%-------------------------------------------------operating conditions
% [OPERATING_CONDITIONS]
vp.tyre.INFLPRES                 = 125000              ; % Tyre inflation pressure
vp.tyre.NOMPRES                  = 125000              ; % Nominal tyre inflation pressure
%----------------------------------------------------------------inertia
% [INERTIA]
vp.tyre.MASS                     = 6.4658              ; % Tyre Mass
vp.tyre.IXX                      = 0.17175             ; % Tyre diametral moment of inertia
vp.tyre.IYY                      = 0.3046              ; % Tyre polar moment of inertia
vp.tyre.BELT_MASS                = 4.5633              ; % Belt mass
vp.tyre.BELT_IXX                 = 0.1445              ; % Belt diametral moment of inertia
vp.tyre.BELT_IYY                 = 0.17719             ; % Belt polar moment of inertia
vp.tyre.GRAVITY                  = -9.81               ; % Gravity acting on belt in Z direction
%------------------------------------------------------------vertical
% [VERTICAL]
vp.tyre.FNOMIN                   = 1000                ; % Nominal wheel load
vp.tyre.VERTICAL_STIFFNESS       = 75831.641           ; % Tyre vertical stiffness %%***CREDO CHE QUESTO SIA UNA CAZZATA***%%
%vp.tyre.VERTICAL_STIFFNESS       = 110070              ; % Tyre vertical stiffness %%***IN CASO QUESTA VIENE DAI DATI SUPER SEGRETI PIRELLI***%%
vp.tyre.VERTICAL_DAMPING         = 50                  ; % Tyre vertical damping
vp.tyre.MC_CONTOUR_A             = 0.5                 ; % Motorcycle contour ellips A
vp.tyre.MC_CONTOUR_B             = 0.5                 ; % Motorcycle contour ellips B
vp.tyre.BREFF                    = 7.2013              ; % Low load stiffness e.r.r.
vp.tyre.DREFF                    = 0.50077             ; % Peak value of e.r.r.
vp.tyre.FREFF                    = 0.052621            ; % High load stiffness e.r.r.
vp.tyre.Q_RE0                    = 1.0246              ; % Ratio of free tyre radius with nominal tyre radius
vp.tyre.Q_V1                     = 0                   ; % Tyre radius increase with speed
vp.tyre.Q_V2                     = 0                   ; % Vertical stiffness increase with speed
vp.tyre.Q_FZ2                    = 0.0001              ; % Quadratic term in load vs. deflection
vp.tyre.Q_FCX                    = 0.065472            ; % Longitudinal force influence on vertical stiffness
vp.tyre.Q_FCY                    = 0.19039             ; % Lateral force influence on vertical stiffness
vp.tyre.Q_CAM                    = 0                   ; % Stiffness reduction due to camber
vp.tyre.PFZ1                     = 0.5459              ; % Pressure effect on vertical stiffness
vp.tyre.BOTTOM_OFFST             = 0.01                ; % Distance to rim when bottoming starts to occur
vp.tyre.BOTTOM_STIFF             = 2000000             ; % Vertical stiffness of bottomed tyre
%-------------------------------------------------------------structural
% [STRUCTURAL]
vp.tyre.LONGITUDINAL_STIFFNESS   = 121330.6257         ; % Tyre overall longitudinal stiffness
vp.tyre.LATERAL_STIFFNESS        = 67795.5646          ; % Tyre overall lateral stiffness
vp.tyre.YAW_STIFFNESS            = 2018.2055           ; % Tyre overall yaw stiffness
vp.tyre.FREQ_LONG                = 49.329              ; % Undamped frequency fore/aft and vertical mode
vp.tyre.FREQ_LAT                 = 40.6285             ; % Undamped frequency lateral mode
vp.tyre.FREQ_YAW                 = 36.8461             ; % Undamped frequency yaw and camber mode
vp.tyre.FREQ_WINDUP              = 42.3721             ; % Undamped frequency wind-up mode
vp.tyre.DAMP_LONG                = 0.04                ; % Dimensionless damping fore/aft and vertical mode
vp.tyre.DAMP_LAT                 = 0.04                ; % Dimensionless damping lateral mode
vp.tyre.DAMP_YAW                 = 0.04                ; % Dimensionless damping yaw and camber mode
vp.tyre.DAMP_WINDUP              = 0.04                ; % Dimensionless damping wind-up mode
vp.tyre.DAMP_RESIDUAL            = 0.002               ; % Residual damping (proportional to stiffness)
vp.tyre.DAMP_VLOW                = 0.001               ; % Additional low speed damping (proportional to stiffness)
vp.tyre.Q_BVX                    = 0                   ; % Load and speed influence on in-plane translation stiffness
vp.tyre.Q_BVT                    = 0                   ; % Load and speed influence on in-plane rotation stiffness
vp.tyre.PCFX1                    = 0                   ; % Tyre overall longitudinal stiffness vertical deflection dependency linear term
vp.tyre.PCFX2                    = 0                   ; % Tyre overall longitudinal stiffness vertical deflection dependency quadratic term
vp.tyre.PCFX3                    = 0                   ; % Tyre overall longitudinal stiffness pressure dependency
vp.tyre.PCFY1                    = -0.074019           ; % Tyre overall lateral stiffness vertical deflection dependency linear term
vp.tyre.PCFY2                    = -1.2876e-07         ; % Tyre overall lateral stiffness vertical deflection dependency quadratic term
vp.tyre.PCFY3                    = 2.4963e-08          ; % Tyre overall lateral stiffness pressure dependency
vp.tyre.PCMZ1                    = 0                   ; % Tyre overall yaw stiffness pressure dependency
%--------------------------------------------------------contact_patch
% [CONTACT_PATCH]
vp.tyre.Q_RA1                    = 0.67                ; % Square root term in contact length equation
vp.tyre.Q_RA2                    = 0.76                ; % Linear term in contact length equation
vp.tyre.Q_RB1                    = 1.24                ; % Root term in contact width equation
vp.tyre.Q_RB2                    = -1.67               ; % Linear term in contact width equation
vp.tyre.ELLIPS_SHIFT             = 0.83                ; % Scaling of distance between front and rear ellipsoid
vp.tyre.ELLIPS_LENGTH            = 1.3                 ; % Semi major axis of ellipsoid
vp.tyre.ELLIPS_HEIGHT            = 1                   ; % Semi minor axis of ellipsoid
vp.tyre.ELLIPS_ORDER             = 1.5                 ; % Order of ellipsoid
vp.tyre.ELLIPS_MAX_STEP          = 0.025               ; % Maximum height of road step
vp.tyre.ELLIPS_NWIDTH            = 10                  ; % Number of parallel ellipsoids
vp.tyre.ELLIPS_NLENGTH           = 10                  ; % Number of ellipsoids at sides of contact patch
%---------------------------------------------inflation_pressure_range
% [INFLATION_PRESSURE_RANGE]
vp.tyre.PRESMIN                  = 70000               ; % Minimum valid tyre inflation pressure
vp.tyre.PRESMAX                  = 195000              ; % Minimum valid tyre inflation pressure
%-------------------------------------------------vertical_force_range
% [VERTICAL_FORCE_RANGE]
vp.tyre.FZMIN                    = 100                 ; % Minimum allowed wheel load
vp.tyre.FZMAX                    = 1919.7952           ; % Maximum allowed wheel load
%------------------------------------------------------long_slip_range
% [LONG_SLIP_RANGE]
vp.tyre.KPUMIN                   = -1.5                ; % Minimum valid wheel slip
vp.tyre.KPUMAX                   = 1.5                 ; % Maximum valid wheel slip
%-----------------------------------------------------slip_angle_range
% [SLIP_ANGLE_RANGE]
vp.tyre.ALPMIN                   = -1.5                ; % Minimum valid slip angle
vp.tyre.ALPMAX                   = 1.5                 ; % Maximum valid slip angle
%-----------------------------------------------inclination_slip_range
% [INCLINATION_ANGLE_RANGE]
vp.tyre.CAMMIN                   = -0.17453            ; % Minimum valid camber angle
vp.tyre.CAMMAX                   = 0.17453             ; % Maximum valid camber angle
%--------------------------------------------------------------scaling
% [SCALING_COEFFICIENTS]
vp.tyre.LFZO                     = 1                   ; % Scale factor of nominal (rated) load
vp.tyre.LCX                      = 1                   ; % Scale factor of Fx shape factor
vp.tyre.LMUX                     = 1                   ; % Scale factor of Fx peak friction coefficient
vp.tyre.LEX                      = 1                   ; % Scale factor of Fx curvature factor
vp.tyre.LKX                      = 1                   ; % Scale factor of Fx slip stiffness
vp.tyre.LHX                      = 1                   ; % Scale factor of Fx horizontal shift
vp.tyre.LVX                      = 1                   ; % Scale factor of Fx vertical shift
vp.tyre.LCY                      = 1                   ; % Scale factor of Fy shape factor
vp.tyre.LMUY                     = 1                   ; % Scale factor of Fy peak friction coefficient
vp.tyre.LEY                      = 1                   ; % Scale factor of Fy curvature factor
vp.tyre.LKY                      = 1                   ; % Scale factor of Fy cornering stiffness
vp.tyre.LKYC                     = 1                   ; % Scale factor of Fy camber stiffness
vp.tyre.LKZC                     = 1                   ; % Scale factor of Mz camber stiffness
vp.tyre.LHY                      = 1                   ; % Scale factor of Fy horizontal shift
vp.tyre.LVY                      = 1                   ; % Scale factor of Fy vertical shift
vp.tyre.LTR                      = 1                   ; % Scale factor of Peak of pneumatic trail
vp.tyre.LRES                     = 1                   ; % Scale factor for offset of residual torque
vp.tyre.LXAL                     = 1                   ; % Scale factor of alpha influence on Fx
vp.tyre.LYKA                     = 1                   ; % Scale factor of kappa influence on Fx
vp.tyre.LVYKA                    = 1                   ; % Scale factor of kappa induced Fy
vp.tyre.LS                       = 1                   ; % Scale factor of Moment arm of Fx
vp.tyre.LMX                      = 1                   ; % Scale factor of overturning couple
vp.tyre.LVMX                     = 1                   ; % Scale factor of Mx vertical shift
vp.tyre.LMY                      = 1                   ; % Scale factor of rolling resistance torque
vp.tyre.LMP                      = 1                   ; % Scale factor of Mz parking torque
%---------------------------------------------------------longitudinal
% [LONGITUDINAL_COEFFICIENTS]
vp.tyre.PCX1                     = 1.1421              ; % Shape factor Cfx for longitudinal force
vp.tyre.PDX1                     = 1.185          ; % Longitudinal friction Mux at Fznom  (1.8222)
vp.tyre.PDX2                     = -0.017189           ; % Variation of friction Mux with load
vp.tyre.PDX3                     = -0.0025953          ; % Variation of friction Mux with camber
vp.tyre.PEX1                     = 0.55262             ; % Longitudinal curvature Efx at Fznom
vp.tyre.PEX2                     = -0.37098            ; % Variation of curvature Efx with load
vp.tyre.PEX3                     = -0.29373            ; % Variation of curvature Efx with load squared
vp.tyre.PEX4                     = -0.50754            ; % Factor in curvature Efx while driving
vp.tyre.PKX1                     = 74.7123             ; % Longitudinal slip stiffness Kfx/Fz at Fznom
vp.tyre.PKX2                     = -0.8902             ; % Variation of slip stiffness Kfx/Fz with load
vp.tyre.PKX3                     = 0                   ; % Exponent in slip stiffness Kfx/Fz with load
vp.tyre.PHX1                     = -0.00073801         ; % Horizontal shift Shx at Fznom
vp.tyre.PHX2                     = -0.00161            ; % Variation of shift Shx with load
vp.tyre.PVX1                     = 0.083707            ; % Vertical shift Svx/Fz at Fznom
vp.tyre.PVX2                     = 0.094794            ; % Variation of shift Svx/Fz with load
vp.tyre.PPX1                     = -1.0462             ; % linear influence of inflation pressure on longitudinal slip stiffness
vp.tyre.PPX2                     = 0.49429             ; % quadratic influence of inflation pressure on longitudinal slip stiffness
vp.tyre.PPX3                     = -0.083526           ; % linear influence of inflation pressure on peak longitudinal friction
vp.tyre.PPX4                     = 0.77186             ; % quadratic influence of inflation pressure on peak longitudinal friction
vp.tyre.RBX1                     = 30.6866             ; % Slope factor for combined slip Fx reduction
vp.tyre.RBX2                     = 38.8391             ; % Variation of slope Fx reduction with kappa
vp.tyre.RBX3                     = 16.5542             ; % Influence of camber on stiffness for Fx combined
vp.tyre.RCX1                     = 0.88971             ; % Shape factor for combined slip Fx reduction
vp.tyre.REX1                     = -1.6419             ; % Curvature factor of combined Fx
vp.tyre.REX2                     = -0.96524            ; % Curvature factor of combined Fx with load
vp.tyre.RHX1                     = -0.0076045          ; % Shift factor for combined slip Fx reduction
%----------------------------------------------------------overturning
% [OVERTURNING_COEFFICIENTS]
vp.tyre.QSX1                     = -0.099993           ; % Lateral force induced overturning moment
vp.tyre.QSX2                     = 0.70614             ; % Camber induced overturning couple
vp.tyre.QSX3                     = -0.18417            ; % Fy induced overturning couple
vp.tyre.QSX4                     = 16.7607             ; % Mixed load lateral force and camber on Mx
vp.tyre.QSX5                     = 1.0146              ; % Load effect on Mx with lateral force and camber
vp.tyre.QSX6                     = 2.3724              ; % B-factor of load with Mx
vp.tyre.QSX7                     = 0.10248             ; % Camber with load on Mx
vp.tyre.QSX8                     = -0.9337             ; % Lateral force with load on Mx
vp.tyre.QSX9                     = 0.13939             ; % B-factor of lateral force with load on Mx
vp.tyre.QSX10                    = -0.90483            ; % Vertical force with camber on Mx
vp.tyre.QSX11                    = 0.39762             ; % B-factor of vertical force with camber on Mx
vp.tyre.QSX12                    = 0.96442             ; % Camber squared induced overturning moment
vp.tyre.QSX13                    = 0.39634             ; % Lateral force induced overturning moment
vp.tyre.QSX14                    = 0.014507            ; % Lateral force induced overturning moment with camber
vp.tyre.PPMX1                    = -0.97018            ; % Influence of inflation pressure on overturning moment
%--------------------------------------------------------------lateral
% [LATERAL_COEFFICIENTS]
vp.tyre.PCY1                     = 1.1962              ; % Shape factor Cfy for lateral forces
vp.tyre.PDY1                     = 1.41              ; % Lateral friction Muy  (1.7385)  
vp.tyre.PDY2                     = -0.28225            ; % Variation of friction Muy with load
vp.tyre.PDY3                     = 0.39173             ; % Variation of friction Muy with squared camber
vp.tyre.PEY1                     = 0.48075             ; % Lateral curvature Efy at Fznom
vp.tyre.PEY2                     = -0.039934           ; % Variation of curvature Efy with load
vp.tyre.PEY3                     = 0.25989             ; % Zero order camber dependency of curvature Efy
vp.tyre.PEY4                     = 0.21047             ; % Variation of curvature Efy with camber
vp.tyre.PEY5                     = -8.9286             ; % Variation of curvature Efy with camber squared
vp.tyre.PKY1                     = -43.0735            ; % Maximum value of stiffness Kfy/Fznom
vp.tyre.PKY2                     = 1.7884              ; % Load at which Kfy reaches maximum value
vp.tyre.PKY3                     = 0.17201             ; % Variation of Kfy/Fznom with camber
vp.tyre.PKY4                     = 2                   ; % Curvature of stiffness Kfy
vp.tyre.PKY5                     = 0.027993            ; % Peak stiffness variation with camber squared
vp.tyre.PKY6                     = -1.3356             ; % Fy camber stiffness factor
vp.tyre.PKY7                     = 0.23917             ; % Vertical load dependency of camber stiffness
vp.tyre.PHY1                     = -0.0035314          ; % Horizontal shift Shy at Fznom
vp.tyre.PHY2                     = -0.0011409          ; % Variation of shift Shy with load
vp.tyre.PVY1                     = -0.07786            ; % Vertical shift in Svy/Fz at Fznom
vp.tyre.PVY2                     = 0.071018            ; % Variation of shift Svy/Fz with load
vp.tyre.PVY3                     = -1.1247             ; % Variation of shift Svy/Fz with camber
vp.tyre.PVY4                     = -0.74656            ; % Variation of shift Svy/Fz with camber and load
vp.tyre.PPY1                     = 0.13515             ; % influence of inflation pressure on cornering stiffness
vp.tyre.PPY2                     = 1.4647              ; % influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
vp.tyre.PPY3                     = 0.066632            ; % linear influence of inflation pressure on lateral peak friction
vp.tyre.PPY4                     = -0.48119            ; % quadratic influence of inflation pressure on lateral peak friction
vp.tyre.PPY5                     = -0.76945            ; % Influence of inflation pressure on camber stiffness
vp.tyre.RBY1                     = 14.791              ; % Slope factor for combined Fy reduction
vp.tyre.RBY2                     = 13.4567             ; % Variation of slope Fy reduction with alpha
vp.tyre.RBY3                     = 0.03792             ; % Shift term for alpha in slope Fy reduction
vp.tyre.RBY4                     = 26.6786             ; % Influence of camber on stiffness of Fy combined
vp.tyre.RCY1                     = 0.95064             ; % Shape factor for combined Fy reduction
vp.tyre.REY1                     = -1.0309             ; % Curvature factor of combined Fy
vp.tyre.REY2                     = 1.2906              ; % Curvature factor of combined Fy with load
vp.tyre.RHY1                     = -0.00021546         ; % Shift factor for combined Fy reduction
vp.tyre.RHY2                     = -0.00078225         ; % Shift factor for combined Fy reduction with load
vp.tyre.RVY1                     = 0.013365            ; % Kappa induced side force Svyk/Muy*Fz at Fznom
vp.tyre.RVY2                     = -0.002718           ; % Variation of Svyk/Muy*Fz with load
vp.tyre.RVY3                     = 1.1393              ; % Variation of Svyk/Muy*Fz with camber
vp.tyre.RVY4                     = 15.0057             ; % Variation of Svyk/Muy*Fz with alpha
vp.tyre.RVY5                     = 1.9085              ; % Variation of Svyk/Muy*Fz with kappa
vp.tyre.RVY6                     = 16.5046             ; % Variation of Svyk/Muy*Fz with atan(kappa)
%---------------------------------------------------rolling resistance
% [ROLLING_COEFFICIENTS]
vp.tyre.QSY1                     = 0.01                ; % Rolling resistance torque coefficient
vp.tyre.QSY2                     = 0                   ; % Rolling resistance torque depending on Fx
vp.tyre.QSY3                     = 0.0004              ; % Rolling resistance torque depending on speed
vp.tyre.QSY4                     = 4e-05               ; % Rolling resistance torque depending on speed ^4
vp.tyre.QSY5                     = 0                   ; % Rolling resistance torque depending on camber squared
vp.tyre.QSY6                     = 0                   ; % Rolling resistance torque depending on load and camber squared
vp.tyre.QSY7                     = 0.85                ; % Rolling resistance torque coefficient load dependency
vp.tyre.QSY8                     = -0.4                ; % Rolling resistance torque coefficient pressure dependency
%-------------------------------------------------------------aligning
% [ALIGNING_COEFFICIENTS]
vp.tyre.QBZ1                     = 17.0993             ; % Trail slope factor for trail Bpt at Fznom
vp.tyre.QBZ2                     = -14.9875            ; % Variation of slope Bpt with load
vp.tyre.QBZ3                     = -1.0802             ; % Variation of slope Bpt with load squared
vp.tyre.QBZ4                     = -2.5127             ; % Variation of slope Bpt with camber
vp.tyre.QBZ5                     = -1.8347             ; % Variation of slope Bpt with absolute camber
vp.tyre.QBZ9                     = 1.0788              ; % Slope factor Br of residual torque Mzr
vp.tyre.QBZ10                    = 0.048647            ; % Slope factor Br of residual torque Mzr
vp.tyre.QCZ1                     = 1.5996              ; % Shape factor Cpt for pneumatic trail
vp.tyre.QDZ1                     = 0.050021            ; % Peak trail Dpt" = Dpt*(Fz/Fznom*R0)
vp.tyre.QDZ2                     = -0.0064409          ; % Variation of peak Dpt" with load
vp.tyre.QDZ3                     = -0.12121            ; % Variation of peak Dpt" with camber
vp.tyre.QDZ4                     = 1.5204              ; % Variation of peak Dpt" with camber squared
vp.tyre.QDZ6                     = 0.041201            ; % Peak residual torque Dmr" = Dmr/(Fz*R0)
vp.tyre.QDZ7                     = -0.010443           ; % Variation of peak factor Dmr" with load
vp.tyre.QDZ8                     = -0.53062            ; % Variation of peak factor Dmr" with camber
vp.tyre.QDZ9                     = 0.36312             ; % Variation of peak factor Dmr" with camber and load
vp.tyre.QDZ10                    = 3.4119              ; % Variation of peak factor Dmr with camber squared
vp.tyre.QDZ11                    = -4.708              ; % Variation of Dmr with camber squared and load
vp.tyre.QEZ1                     = 0.49582             ; % Trail curvature Ept at Fznom
vp.tyre.QEZ2                     = 8.2761              ; % Variation of curvature Ept with load
vp.tyre.QEZ3                     = -19.9006            ; % Variation of curvature Ept with load squared
vp.tyre.QEZ4                     = 0.81291             ; % Variation of curvature Ept with sign of Alpha-t
vp.tyre.QEZ5                     = 7.0363              ; % Variation of Ept with camber and sign Alpha-t
vp.tyre.QHZ1                     = -0.029989           ; % Trail horizontal shift Sht at Fznom
vp.tyre.QHZ2                     = 0.029789            ; % Variation of shift Sht with load
vp.tyre.QHZ3                     = 0.28844             ; % Variation of shift Sht with camber
vp.tyre.QHZ4                     = -0.39318            ; % Variation of shift Sht with camber and load
vp.tyre.PPZ1                     = 1.6157              ; % effect of inflation pressure on length of pneumatic trail
vp.tyre.PPZ2                     = -0.99601            ; % Influence of inflation pressure on residual aligning torque
vp.tyre.SSZ1                     = 0.064778            ; % Nominal value of s/R0: effect of Fx on Mz
vp.tyre.SSZ2                     = 0.023643            ; % Variation of distance s/R0 with Fy/Fznom
vp.tyre.SSZ3                     = 1.3152              ; % Variation of distance s/R0 with camber
vp.tyre.SSZ4                     = -0.92921            ; % Variation of distance s/R0 with load and camber
%-------------------------------------------------------------turnslip
% [TURNSLIP_COEFFICIENTS]
vp.tyre.PDXP1                    = 0.4                 ; % Peak Fx reduction due to spin parameter
vp.tyre.PDXP2                    = 0                   ; % Peak Fx reduction due to spin with varying load parameter
vp.tyre.PDXP3                    = 0                   ; % Peak Fx reduction due to spin with kappa parameter
vp.tyre.PKYP1                    = 1                   ; % Cornering stiffness reduction due to spin
vp.tyre.PDYP1                    = 0.4                 ; % Peak Fy reduction due to spin parameter
vp.tyre.PDYP2                    = 0                   ; % Peak Fy reduction due to spin with varying load parameter
vp.tyre.PDYP3                    = 0                   ; % Peak Fy reduction due to spin with alpha parameter
vp.tyre.PDYP4                    = 0                   ; % Peak Fy reduction due to square root of spin parameter
vp.tyre.PHYP1                    = 1                   ; % Fy-alpha curve lateral shift limitation
vp.tyre.PHYP2                    = 0.15                ; % Fy-alpha curve maximum lateral shift parameter
vp.tyre.PHYP3                    = 0                   ; % Fy-alpha curve maximum lateral shift varying with load parameter
vp.tyre.PHYP4                    = -4                  ; % Fy-alpha curve maximum lateral shift parameter
vp.tyre.PECP1                    = 0.5                 ; % Camber w.r.t. spin reduction factor parameter in camber stiffness
vp.tyre.PECP2                    = 0                   ; % Camber w.r.t. spin reduction factor varying with load parameter in camber stiffness
vp.tyre.QDTP1                    = 10                  ; % Pneumatic trail reduction factor due to turn slip parameter
vp.tyre.QCRP1                    = 0.2                 ; % Turning moment at constant turning and zero forward speed parameter
vp.tyre.QCRP2                    = 0.1                 ; % Turn slip moment (at alpha=90deg) parameter for increase with spin
vp.tyre.QBRP1                    = 0.1                 ; % Residual (spin) torque reduction factor parameter due to side slip
vp.tyre.QDRP1                    = 1                   ; % Turn slip moment peak magnitude parameter

%% Powertrain
% Oh la gioia
% Allora, per adesso ci sta bisogno di metterci su potenza massima, coppia
% motrice massima e coppia frenante massima, io una vaga idea di quanto
% possano essere ce l'ho ma è sicuramente da riguardare, da riscrivere
% persino non appena si capisce qualcosa di powertrain
vp.Pmax = 2*60e3; % [W] potenza limite da regolamento
vp.Tdrivemax = 400; % [Nm] a regola, vera, falsa? BOOOH, credo che quella del motore sia 85 e i 5.3 sono dal rapporto al ponte ma BOOOOOOOOOH
vp.Tbrakemax = 8000; % [Nm], SEH COME NO, numero grande perché è una roba difficile da stimare, i freni non si rifanno da na vita ma generalmente ci si aspetta che freni, se vengono sparati 12 kNm di frenata però magari controlliamo eh?
vp.BB = 0.4; % [], brake balance se non è cambiato o ho sbagliato lato macchina :)))
vp.Vmax = roots([-0.5*vp.rho*vp.Cx*vp.S,0,-vp.f*vp.m*vp.g,vp.Pmax]); % Metodo carino di calcolarsi la massima velocità del veicolo tenendo conto di resistenza aerodinamica e di rotolamento (ma tanto sta macchina non ha mai fatto più di 70 all'ora...)
vp.Vmax = vp.Vmax(find(vp.Vmax==real(vp.Vmax),1)); % [m/s], questa è la velocità massima (da portare nei reali se il roots ha fatto danni)
vp.Tregenmax = 25; % [Nm]