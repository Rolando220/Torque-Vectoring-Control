% %% TUNING ANALITICO DEL PI (Cancellazione Polo-Zero)
% % Dati veicolo
% m = vp.m; Jz = vp.Jz; a1 = vp.a1; a2 = vp.a2;
% Caf = 1150 * (180/pi); Car = 1010 * (180/pi);
% 
% Vx = 1; % Analizziamo a 10 m/s (36 km/h)
% 
% % Matrici del Modello Lineare (A e B)
% A11 = -(Caf + Car) / (m * Vx);
% A12 = ((Car * a2 - Caf * a1) / (m * Vx)) - Vx;
% A21 = (Car * a2 - Caf * a1) / (Jz * Vx);
% A22 = -(Caf * a1^2 + Car * a2^2) / (Jz * Vx);
% 
% B11 = Caf / m;            
% B12 = 0;                  
% B21 = (Caf * a1) / Jz;    
% B22 = 1 / Jz;             
% 
% A = [A11, A12; A21, A22];
% B = [B11, B12; B21, B22];
% C = [0, 1]; % Uscita: Yaw Rate (r)
% D = [0, 0];
% 
% % Funzione di Trasferimento del veicolo
% sys = ss(A, B, C, D);
% P_Mz_to_r = tf(sys(1, 2));
% 
% %% 1. Calcolo dei Poli
% poli_veicolo = pole(P_Mz_to_r);
% disp('--- POLI DEL VEICOLO A 10 m/s ---');
% disp(poli_veicolo);
% 
% % Troviamo il polo più "lento" (quello più vicino all'asse immaginario)
% polo_lento = max(real(poli_veicolo)); 
% zero_pi = abs(polo_lento); % Vogliamo s + zero_pi = 0
% fprintf('Posizioniamo lo zero del PI in: s = -%.2f\n', zero_pi);
% 
% %% 2. Risposta Indiciale con diversi Kp
% % Sapendo che zero_pi = Ki / Kp, proviamo a variare Kp e calcoliamo Ki di conseguenza
% Kp_test = [1000, 2000, 3000]; 
% 
% figure('Name', 'Analisi Risposta Indiciale TVC');
% hold on; grid on;
% 
% for i = 1:length(Kp_test)
%     Kp = Kp_test(i);
%     Ki = Kp * zero_pi; % Il legame analitico perfetto
% 
%     % Creiamo il controllore PI: C(s) = (Kp*s + Ki)/s
%     C_pi = tf([Kp, Ki], [1, 0]); 
% 
%     % Anello Chiuso: L(s) = C(s)*P(s) -> T(s) = L(s)/(1+L(s))
%     L = C_pi * P_Mz_to_r;
%     T = feedback(L, 1);
% 
%     % Plot della risposta al gradino
%     step(T, 1.5); % Simuliamo per 1.5 secondi
% 
%     fprintf('Test %d: Kp = %d, Ki = %.1f\n', i, Kp, Ki);
% end
% 
% title('Risposta Indiciale ad Anello Chiuso (V_x = 10 m/s)');
% legend(sprintf('Kp = %d', Kp_test(1)), ...
%        sprintf('Kp = %d', Kp_test(2)), ...
%        sprintf('Kp = %d', Kp_test(3)), 'Location', 'best');

%% GAIN SCHEDULING ANALITICO (Cancellazione Polo-Zero)
% Assicurati di avere la struct vp caricata

m = vp.m; Jz = vp.Jz; a1 = vp.a1; a2 = vp.a2;
Caf = 1150 * (180/pi); Car = 1010 * (180/pi);

Vx_array = 1:1:30; % Range di velocità [m/s]
Kp_array = zeros(size(Vx_array));
Ki_array = zeros(size(Vx_array));

% Fissiamo il Kp che ci ha dato la dinamica ottimale (es. 0.2s di assestamento)
Kp_target = 1000; 

disp('--- GAIN SCHEDULING: TABELLA LUT ---');

for i = 1:length(Vx_array)
    Vx = Vx_array(i);

    % Ricostruiamo la Matrice A per la velocità corrente
    A11 = -(Caf + Car) / (m * Vx);
    A12 = ((Car * a2 - Caf * a1) / (m * Vx)) - Vx;
    A21 = (Car * a2 - Caf * a1) / (Jz * Vx);
    A22 = -(Caf * a1^2 + Car * a2^2) / (Jz * Vx);

    A = [A11, A12; A21, A22];

    % Calcoliamo gli autovalori (i poli)
    poli_veicolo = eig(A);

    % Troviamo il polo dominante (più lento, quello più vicino a 0)
    polo_lento = max(real(poli_veicolo)); 
    zero_pi = abs(polo_lento);

    % Salviamo i guadagni perfetti
    Kp_array(i) = Kp_target;
    Ki_array(i) = Kp_target * zero_pi;

    fprintf('Vx = %2d m/s -> Polo Lento = %6.2f | Kp = %4d | Ki = %8.1f\n', ...
            Vx, polo_lento, Kp_array(i), Ki_array(i));
end

%% Plot delle LUT
figure('Name', 'Look-Up Tables TVC');
plot(Vx_array, Ki_array, '-o', 'LineWidth', 2, 'Color', 'r');
grid on; title('Andamento K_i (Zero Tracking) vs Velocità');
xlabel('Velocità V_x [m/s]'); ylabel('K_i Mappato');