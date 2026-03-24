%% ========================================================================
%% 1. GAIN SCHEDULING ANALITICO E CREAZIONE LUT
%% ========================================================================
% Dati veicolo
m = vp.m; Jz = vp.Jz; a1 = vp.a1; a2 = vp.a2;
Caf = 1150 * (180/pi); Car = 1010 * (180/pi);
C = [0, 1]; D = [0, 0];

Vx_array = 1:1:30; % Range di velocità [m/s]
Kp_array = zeros(size(Vx_array));
Ki_array = zeros(size(Vx_array));
Kp_target = 1000; 

disp('--- GAIN SCHEDULING: CALCOLO LUT ---');
for i = 1:length(Vx_array)
    Vx = Vx_array(i);
    A = [-(Caf + Car)/(m * Vx),          ((Car * a2 - Caf * a1)/(m * Vx)) - Vx; ...
         (Car * a2 - Caf * a1)/(Jz * Vx), -(Caf * a1^2 + Car * a2^2)/(Jz * Vx)];
    
    polo_lento = max(real(eig(A))); 
    
    Kp_array(i) = Kp_target;
    Ki_array(i) = Kp_target * abs(polo_lento);
end
disp('LUT Calcolate con successo!');

%% ========================================================================
%% 2. ANALISI LINEARE MULTIPLA (Bode, Sensitività, Gradino)
%% ========================================================================
disp('--- AVVIO ANALISI MULTIPLA SU DIVERSE VELOCITA'' ---');
Vx_test_array = 5:5:30; 
legend_labels = cell(1, length(Vx_test_array));

fig_bode = figure('Name', 'Bode Loop Aperto L(s)'); hold on; grid on;
fig_sens = figure('Name', 'Funzione di Sensitività S(s)'); hold on; grid on;
fig_step = figure('Name', 'Risposta al Gradino T(s)'); hold on; grid on;

for i = 1:length(Vx_test_array)
    Vx_t = Vx_test_array(i);
    legend_labels{i} = sprintf('V_x = %d m/s', Vx_t);
    
    % Impianto
    A_t = [-(Caf + Car)/(m * Vx_t),          ((Car * a2 - Caf * a1)/(m * Vx_t)) - Vx_t; ...
           (Car * a2 - Caf * a1)/(Jz * Vx_t), -(Caf * a1^2 + Car * a2^2)/(Jz * Vx_t)];
    B_t = [Caf/m, 0; (Caf*a1)/Jz, 1/Jz];
    
    P_t = tf(ss(A_t, B_t, C, D));
    P_t = P_t(1, 2); 
    
    % Controllore prelevato dalla LUT!
    Ki_t = interp1(Vx_array, Ki_array, Vx_t); 
    C_pi = tf([Kp_target, Ki_t], [1, 0]);
    
    % Anelli
    L_t = C_pi * P_t;
    S_t = 1 / (1 + L_t);
    T_t = feedback(L_t, 1);
    
    figure(fig_bode); margin(L_t); 
    figure(fig_sens); bodemag(S_t, {0.1, 1000});
    figure(fig_step); step(T_t, 1.5);
end

figure(fig_bode); title('Bode Ad Anello Aperto L(s) vs V_x'); legend(legend_labels, 'Location', 'best');
figure(fig_sens); title('Funzione di Sensitività S(s) vs V_x'); legend(legend_labels, 'Location', 'best');
figure(fig_step); title('Risposta al Gradino T(s) vs V_x'); legend(legend_labels, 'Location', 'best');

%% ========================================================================
%% --- SETUP NOMINALE PER ANALISI DI ROBUSTEZZA (V_x = 25 m/s) ---
%% ========================================================================
Vx_rob = 25; 
% Pesco il K_i dalla LUT per simulare il comportamento reale della centralina
Ki_rob = interp1(Vx_array, Ki_array, Vx_rob);
C_pi_rob = tf([Kp_target, Ki_rob], [1, 0]);

A_nom = [-(Caf + Car)/(m * Vx_rob),          ((Car * a2 - Caf * a1)/(m * Vx_rob)) - Vx_rob; ...
         (Car * a2 - Caf * a1)/(Jz * Vx_rob), -(Caf * a1^2 + Car * a2^2)/(Jz * Vx_rob)];
B_nom = [Caf/m, 0; (Caf*a1)/Jz, 1/Jz];

P_nom_rob = tf(ss(A_nom, B_nom, C, D));
P_nom_rob = P_nom_rob(1, 2);

%% 3. ROBUSTEZZA: Variazione Aderenza C_alpha
disp('--- ANALISI ROBUSTEZZA GOMME ---');
var_array = [0.7, 1.0, 1.3]; 
labels_rob = {'C_\alpha -30% (Usurate)', 'Nominale (100%)', 'C_\alpha +30% (Calde)'};
figure('Name', 'Robustezza Aderenza'); hold on; grid on;

for idx = 1:length(var_array)
    molt = var_array(idx);
    Caf_v = Caf * molt; Car_v = Car * molt;
    
    A_var = [-(Caf_v+Car_v)/(m*Vx_rob), ((Car_v*a2-Caf_v*a1)/(m*Vx_rob))-Vx_rob; ...
             (Car_v*a2-Caf_v*a1)/(Jz*Vx_rob), -(Caf_v*a1^2+Car_v*a2^2)/(Jz*Vx_rob)];
    B_var = [Caf_v/m, 0; (Caf_v*a1)/Jz, 1/Jz];
    
    P_var = tf(ss(A_var, B_var, C, D));
    
    step(feedback(C_pi_rob * P_var(1, 2), 1), 1.5);
end
title(sprintf('Robustezza alle Variazioni di Aderenza (V_x = %d m/s)', Vx_rob));
legend(labels_rob, 'Location', 'best');

%% 4. ROBUSTEZZA: Variazione Massa Pilota
disp('--- ANALISI ROBUSTEZZA MASSA PILOTA ---');
var_massa = [0.90, 1.0, 1.10]; 
labels_massa = {'Pilota Leggero (-10%)', 'Nominale (300 kg)', 'Pilota Pesante (+10%)'};
figure('Name', 'Robustezza Massa'); hold on; grid on;

for idx = 1:length(var_massa)
    molt_m = var_massa(idx);
    m_v = m * molt_m; Jz_v = Jz * molt_m;
    
    A_var = [-(Caf+Car)/(m_v*Vx_rob), ((Car*a2-Caf*a1)/(m_v*Vx_rob))-Vx_rob; ...
             (Car*a2-Caf*a1)/(Jz_v*Vx_rob), -(Caf*a1^2+Car*a2^2)/(Jz_v*Vx_rob)];
    B_var = [Caf/m_v, 0; (Caf*a1)/Jz_v, 1/Jz_v];
    
    P_var = tf(ss(A_var, B_var, C, D));
    
    step(feedback(C_pi_rob * P_var(1, 2), 1), 1.5);
end
title(sprintf('Robustezza alla Variazione di Massa (V_x = %d m/s)', Vx_rob));
legend(labels_massa, 'Location', 'best');

%% 5. ROBUSTEZZA: Ritardo Attuatore (Inverter)
disp('--- ANALISI ROBUSTEZZA RITARDO INVERTER ---');
tau_array = [0, 0.01, 0.02, 0.04]; 
labels_delay = {'Nominale (0 ms)', 'Ritardo Inverter (10 ms)', 'Ritardo Inverter (20 ms)', 'Ritardo Estremo (40 ms)'};
figure('Name', 'Robustezza Ritardo Inverter'); hold on; grid on;

for idx = 1:length(tau_array)
    tau = tau_array(idx);
    
    if tau == 0
        L_var = C_pi_rob * P_nom_rob;
    else
        L_var = C_pi_rob * tf(1, [tau, 1]) * P_nom_rob; % Aggiunta filtro PT1 in serie
    end
    
    step(feedback(L_var, 1), 1.5);
end
title(sprintf('Robustezza al Ritardo dell''Inverter (V_x = %d m/s)', Vx_rob));
legend(labels_delay, 'Location', 'best');

disp('Tutte le analisi concluse!');