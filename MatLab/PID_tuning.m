%% TUNING ANALITICO DEL PI (Cancellazione Polo-Zero)
% Dati veicolo
m = vp.m; Jz = vp.Jz; a1 = vp.a1; a2 = vp.a2;
Caf = 1150 * (180/pi); Car = 1010 * (180/pi);

Vx = 30; % Analizziamo a 10 m/s (36 km/h)

% Matrici del Modello Lineare (A e B)
A11 = -(Caf + Car) / (m * Vx);
A12 = ((Car * a2 - Caf * a1) / (m * Vx)) - Vx;
A21 = (Car * a2 - Caf * a1) / (Jz * Vx);
A22 = -(Caf * a1^2 + Car * a2^2) / (Jz * Vx);

B11 = Caf / m;            
B12 = 0;                  
B21 = (Caf * a1) / Jz;    
B22 = 1 / Jz;             

A = [A11, A12; A21, A22];
B = [B11, B12; B21, B22];
C = [0, 1]; % Uscita: Yaw Rate (r)
D = [0, 0];

% Funzione di Trasferimento del veicolo
sys = ss(A, B, C, D);
P_Mz_to_r = tf(sys(1, 2));

%% 1. Calcolo dei Poli
poli_veicolo = pole(P_Mz_to_r);
disp('--- POLI DEL VEICOLO A 10 m/s ---');
disp(poli_veicolo);

% Troviamo il polo più "lento" (quello più vicino all'asse immaginario)
polo_lento = max(real(poli_veicolo)); 
zero_pi = abs(polo_lento); % Vogliamo s + zero_pi = 0
fprintf('Posizioniamo lo zero del PI in: s = -%.2f\n', zero_pi);

%% 2. Risposta Indiciale con diversi Kp
% Sapendo che zero_pi = Ki / Kp, proviamo a variare Kp e calcoliamo Ki di conseguenza
Kp_test = [1000, 2000, 3000]; 

figure('Name', 'Analisi Risposta Indiciale TVC');
hold on; grid on;

for i = 1:length(Kp_test)
    Kp = Kp_test(i);
    Ki = Kp * zero_pi; % Il legame analitico perfetto

    % Creiamo il controllore PI: C(s) = (Kp*s + Ki)/s
    C_pi = tf([Kp, Ki], [1, 0]); 

    % Anello Chiuso: L(s) = C(s)*P(s) -> T(s) = L(s)/(1+L(s))
    L = C_pi * P_Mz_to_r;
    T = feedback(L, 1);

    % Plot della risposta al gradino
    step(T, 1.5); % Simuliamo per 1.5 secondi

    fprintf('Test %d: Kp = %d, Ki = %.1f\n', i, Kp, Ki);
end

title('Risposta Indiciale ad Anello Chiuso (V_x = 10 m/s)');
legend(sprintf('Kp = %d', Kp_test(1)), ...
       sprintf('Kp = %d', Kp_test(2)), ...
       sprintf('Kp = %d', Kp_test(3)), 'Location', 'best');

%% GAIN SCHEDULING ANALITICO (Cancellazione Polo-Zero)
% Assicurati di avere la struct vp caricata

m = vp.m; Jz = vp.Jz; a1 = vp.a1; a2 = vp.a2;
Caf = 1150 * (180/pi); Car = 1010 * (180/pi);

Vx_array = 1:1:30; % Range di velocità [m/s]
Kp_array = zeros(size(Vx_array));
Ki_array = zeros(size(Vx_array));

% Fissiamo il Kp che ci ha dato la dinamica ottimale (es. 0.2s di assestamento)
Kp_target = 2000; 

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
% figure('Name', 'Look-Up Tables TVC');
% plot(Vx_array, Ki_array, '-o', 'LineWidth', 2, 'Color', 'r');
% grid on; title('Andamento K_i (Zero Tracking) vs Velocità');
% xlabel('Velocità V_x [m/s]'); ylabel('K_i Mappato');

% %% ========================================================================
% %% 2. ANALISI LINEARE MULTIPLA (Bode, Sensitività, Gradino)
% %% ========================================================================
% disp('--- AVVIO ANALISI MULTIPLA SU DIVERSE VELOCITA'' ---');
% 
% % Vettori di test per l'analisi (da 5 a 30 m/s)
% Vx_test_array = 5:5:30; 
% legend_labels = cell(1, length(Vx_test_array));
% 
% % Prepariamo le figure "vuote" e attiviamo hold on
% fig_bode = figure('Name', 'Bode Loop Aperto L(s)'); hold on; grid on;
% fig_sens = figure('Name', 'Funzione di Sensitività S(s)'); hold on; grid on;
% fig_step = figure('Name', 'Risposta al Gradino T(s)'); hold on; grid on;
% 
% for i = 1:length(Vx_test_array)
%     Vx_t = Vx_test_array(i);
%     legend_labels{i} = sprintf('V_x = %d m/s', Vx_t);
% 
%     % 1. Ricostruzione modello nominale a questa velocità
%     A11 = -(Caf + Car) / (m * Vx_t);
%     A12 = ((Car * a2 - Caf * a1) / (m * Vx_t)) - Vx_t;
%     A21 = (Car * a2 - Caf * a1) / (Jz * Vx_t);
%     A22 = -(Caf * a1^2 + Car * a2^2) / (Jz * Vx_t);
% 
%     A_nom = [A11, A12; A21, A22];
%     B_nom = [Caf/m, 0; (Caf*a1)/Jz, 1/Jz];
%     C = [0, 1]; D = [0, 0];
% 
%     sys_nom = ss(A_nom, B_nom, C, D);
%     P_nom = tf(sys_nom(1, 2)); % Pianta da Mz a r
% 
%     % 2. Ricostruzione del controllore esatto per questa velocità
%     polo_lento_t = max(real(eig(A_nom)));
%     Ki_t = Kp_target * abs(polo_lento_t);
%     C_pi = tf([Kp_target, Ki_t], [1, 0]);
% 
%     % 3. Funzioni L(s), S(s), T(s)
%     L_nom = C_pi * P_nom;
%     S_nom = 1 / (1 + L_nom);
%     T_nom = feedback(L_nom, 1);
% 
%     % --- PLOT SULLE FIGURE ---
%     figure(fig_bode); margin(L_nom); 
% 
%     figure(fig_sens); bodemag(S_nom, {0.1, 1000});
% 
%     figure(fig_step); step(T_nom, 1.5);
% end
% 
% % Sistemiamo Titoli e Legende
% figure(fig_bode); title('Bode Ad Anello Aperto L(s) vs V_x'); legend(legend_labels, 'Location', 'best'); grid("on");
% figure(fig_sens); title('Funzione di Sensitività S(s) vs V_x'); legend(legend_labels, 'Location', 'best'); grid("on");
% figure(fig_step); title('Risposta al Gradino T(s) vs V_x'); legend(legend_labels, 'Location', 'best'); grid("on");
% 
% 
% %% ========================================================================
% %% 3. ROBUSTEZZA PARAMETRICA (Variazione Aderenza C_alpha)
% %% ========================================================================
% disp('--- ANALISI ROBUSTEZZA GOMME (V_x = 25 m/s) ---');
% % Fissiamo una velocità critica alta per testare le gomme usurate
% Vx_rob = 25; 
% 
% var_array = [0.7, 1.0, 1.3]; % -30% grip, Nominale, +30% grip
% labels_rob = {'C_\alpha -30% (Usurate)', 'Nominale (100%)', 'C_\alpha +30% (Calde)'};
% 
% figure('Name', 'Robustezza Parametrica al Gradino'); hold on; grid on;
% 
% % Estraiamo il controllore nominale a 25 m/s
% A_nom_rob = [-(Caf+Car)/(m*Vx_rob), ((Car*a2-Caf*a1)/(m*Vx_rob))-Vx_rob; ...
%              (Car*a2-Caf*a1)/(Jz*Vx_rob), -(Caf*a1^2+Car*a2^2)/(Jz*Vx_rob)];
% Ki_rob = Kp_target * abs(max(real(eig(A_nom_rob))));
% C_pi_rob = tf([Kp_target, Ki_rob], [1, 0]);
% 
% for idx = 1:length(var_array)
%     molt = var_array(idx);
% 
%     % Ricalcoliamo la pianta con le gomme modificate
%     Caf_v = Caf * molt; Car_v = Car * molt;
% 
%     A_var = [-(Caf_v+Car_v)/(m*Vx_rob), ((Car_v*a2-Caf_v*a1)/(m*Vx_rob))-Vx_rob; ...
%              (Car_v*a2-Caf_v*a1)/(Jz*Vx_rob), -(Caf_v*a1^2+Car_v*a2^2)/(Jz*Vx_rob)];
%     B_var = [Caf_v/m, 0; (Caf_v*a1)/Jz, 1/Jz];
% 
%     P_var = tf(ss(A_var, B_var, C, D));
%     P_var = P_var(1, 2);
% 
%     % Chiudiamo l'anello usando IL CONTROLLORE NOMINALE
%     T_var = feedback(C_pi_rob * P_var, 1);
% 
%     step(T_var, 1.5);
% end
% 
% title(sprintf('Robustezza alle Variazioni di Aderenza (V_x = %d m/s)', Vx_rob));
% legend(labels_rob, 'Location', 'best');
% 
% %% ========================================================================
% %% 4. ROBUSTEZZA PARAMETRICA (Variazione Massa Pilota)
% %% ========================================================================
% disp('--- ANALISI ROBUSTEZZA MASSA PILOTA (V_x = 25 m/s) ---');
% % Variazione stimata: Pilota leggero (-10% massa) vs Pilota pesante (+10% massa)
% var_massa = [0.90, 1.0, 1.10]; 
% labels_massa = {'Pilota Leggero (-10%)', 'Nominale (300 kg)', 'Pilota Pesante (+10%)'};
% 
% figure('Name', 'Robustezza al Cambio Pilota (Massa e Inerzia)'); hold on; grid on;
% 
% for idx = 1:length(var_massa)
%     molt_m = var_massa(idx);
% 
%     % Se la massa varia, varia proporzionalmente anche l'inerzia d'imbardata Jz!
%     m_v = m * molt_m;
%     Jz_v = Jz * molt_m;
% 
%     % Ricalcoliamo l'impianto
%     A_var = [-(Caf+Car)/(m_v*Vx_rob), ((Car*a2-Caf*a1)/(m_v*Vx_rob))-Vx_rob; ...
%              (Car*a2-Caf*a1)/(Jz_v*Vx_rob), -(Caf*a1^2+Car*a2^2)/(Jz_v*Vx_rob)];
%     B_var = [Caf/m_v, 0; (Caf*a1)/Jz_v, 1/Jz_v];
% 
%     P_var = tf(ss(A_var, B_var, C, D));
%     P_var = P_var(1, 2);
% 
%     % Anello chiuso col controllore NOMINALE
%     T_var = feedback(C_pi_rob * P_var, 1);
% 
%     step(T_var, 1.5);
% end
% title(sprintf('Robustezza alla Variazione di Massa (V_x = %d m/s)', Vx_rob));
% legend(labels_massa, 'Location', 'best');
% 
% %% ========================================================================
% %% 5. ROBUSTEZZA PARAMETRICA (Ritardo Attuatore / Inverter Delay)
% %% ========================================================================
% disp('--- ANALISI ROBUSTEZZA RITARDO INVERTER (V_x = 25 m/s) ---');
% 
% % Costanti di tempo del ritardo da testare (in secondi)
% tau_array = [0, 0.01, 0.02, 0.04]; % 0, 10ms, 20ms, 40ms
% labels_delay = {'Nominale (0 ms)', 'Ritardo Inverter (10 ms)', 'Ritardo Inverter (20 ms)', 'Ritardo Estremo (40 ms)'};
% 
% figure('Name', 'Robustezza al Ritardo Attuatore'); hold on; grid on;
% 
% % Estraiamo il controllore e l'impianto nominale a 25 m/s
% A_nom_rob = [-(Caf+Car)/(m*Vx_rob), ((Car*a2-Caf*a1)/(m*Vx_rob))-Vx_rob; ...
%              (Car*a2-Caf*a1)/(Jz*Vx_rob), -(Caf*a1^2+Car*a2^2)/(Jz*Vx_rob)];
% B_nom_rob = [Caf/m, 0; (Caf*a1)/Jz, 1/Jz];
% 
% P_nom_rob = tf(ss(A_nom_rob, B_nom_rob, C, D));
% P_nom_rob = P_nom_rob(1, 2); % Pianta da Mz a r
% 
% for idx = 1:length(tau_array)
%     tau = tau_array(idx);
% 
%     if tau == 0
%         % Sistema ideale (quello progettato finora)
%         L_var = C_pi_rob * P_nom_rob;
%     else
%         % Aggiungiamo il modello del ritardo (Filtro PT1) in serie
%         G_delay = tf(1, [tau, 1]);
%         L_var = C_pi_rob * G_delay * P_nom_rob;
%     end
% 
%     % Chiudiamo l'anello
%     T_var = feedback(L_var, 1);
% 
%     % Plottiamo la risposta al gradino
%     step(T_var, 1.5);
% end
% 
% title(sprintf('Robustezza al Ritardo dell''Inverter (V_x = %d m/s)', Vx_rob));
% legend(labels_delay, 'Location', 'best');
% 
% disp('Analisi del Ritardo Inverter completata con successo!');
% 
% disp('Tutte le analisi concluse!');