function Kt = ARB(E, Wv, Thv, L, h0, h1)

% funzione per calcolare la rigidezza dell'ARB anteriore
%% Definizione parametri

% Selezione barra e settaggio
Ws = 1;
Ths = 1;

% Barra e settaggio selezionati
W = Wv(Ws);
Th = Thv(Ths);

eps = (h0 - h1)/(L);% Coeff. angolare per altezza di barra


%% Calcoli
x = 0;
H = h0 - eps*x;
% A = W*H;
J1 = (W*H^3)/12;
J2 = (W^3*H)/12;
y1 = zeros(L);
y2 = zeros(L);
y1(x+1) = (L-x)^2/(E*J1);
y2(x+1) = (L-x)^2/(E*J2);
Int1 = 0;
Int2 = 0;
for x = 1 : L

    H = h0 - eps*x;
    % A = W*H;
    J1 = (W*H^3)/12;
    J2 = (W^3*H)/12;

    % Integrazione con trapezi
    y1(x+1) = (L-x)^2/(E*J1);
    y2(x+1) = (L-x)^2/(E*J2);
    Int1 = Int1 + (y1(x)+y1(x+1))/2;
    Int2 = Int2 + (y2(x)+y2(x+1))/2;
end

% Rigidezze coltelli
KflM = 1/Int1;
Kflm = 1/Int2;

% Rigidezza coltello in funzione di Theta (Th) - complessiva
Kfl = (KflM*sin(Th)^2 + Kflm*cos(Th)^2);

% Rigidezza torsionale equivalente
Kt = Kfl*L^2;

end