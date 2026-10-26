%% Script per Grafici di Presentazione: Riferimenti e Actual separati (Linee Continue)

fig = figure('Name', 'Validazione TVC: Simulink vs C-Code', ...
             'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.6], ...
             'Color', 'w');

%% 1. GRAFICO SINISTRO: IL RIFERIMENTO (REFERENCE)
subplot(1, 2, 1);
hold on; grid on;

% Plotta Simulink (Linea Continua molto spessa, Blu Scuro)
plot(out.yaw_ref_sim.Time, squeeze(out.yaw_ref_sim.Data), '-', 'Color', '#00274C', 'LineWidth', 3.5);

% Plotta C Code (Linea Continua spessa, Arancione)
plot(out.yaw_ref_c.Time, squeeze(out.yaw_ref_c.Data), '-', 'Color', '#FF8200', 'LineWidth', 2.0);

% Stile e formattazione
title('Reference Generator: Simulink vs C-Code', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Yaw Rate Reference [rad/s]', 'FontSize', 14);
legend('Simulink (MATLAB)', 'C-Code (C Caller)', 'Location', 'best', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.4);
xlim([0 max(out.yaw_ref_sim.Time)]);

%% 2. GRAFICO DESTRO: L'INSEGUIMENTO (ACTUAL YAW RATE)
subplot(1, 2, 2);
hold on; grid on;

% Plotta Simulink (Linea Continua molto spessa, Verde Scuro)
plot(out.yaw_act_sim.Time, squeeze(out.yaw_act_sim.Data), '-', 'Color', '#005A36', 'LineWidth', 3.5);

% Plotta C Code (Linea Continua spessa, Rosso Corsa)
plot(out.yaw_act_c.Time, squeeze(out.yaw_act_c.Data), '-', 'Color', '#E31837', 'LineWidth', 2.0);

% Stile e formattazione
title('Control Tracking: Actual Yaw Rate', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Yaw Rate [rad/s]', 'FontSize', 14);
legend('Actual (Simulink Control)', 'Actual (C-Code Control)', 'Location', 'best', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2, 'GridAlpha', 0.4);
xlim([0 max(out.yaw_act_sim.Time)]);