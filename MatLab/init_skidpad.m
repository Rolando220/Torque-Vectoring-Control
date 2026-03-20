% --- Parametri del Cerchio ---
R = 9.125;                         % Raggio in metri
circonferenza = 2 * pi * R;     

% --- Generazione Breakpoints Uniformi ---
% Usiamo 200 punti per avere un'ottima precisione (passo ~0.47m)
s_ref = linspace(0, circonferenza, 200); 

% --- Generazione Coordinate (X, Y) e Yaw (Psi) ---
% Calcoliamo l'angolo theta in base alla distanza percorsa s
theta = s_ref / R; 

x_ref = R * cos(theta);     % Coordinata X
y_ref = R * sin(theta);     % Coordinata Y
psi_ref = theta + pi/2;     % Angolo di imbardata (tangente al cerchio)

% --- Verifica Continuità ---
% Forza l'ultimo punto a essere identico al primo per il "Wrap"
x_ref(end) = x_ref(1);
y_ref(end) = y_ref(1);
% Per lo Yaw, il wrap gestisce la rotazione di 2pi automaticamente


