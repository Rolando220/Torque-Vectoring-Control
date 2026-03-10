figure;
plot(squeeze(out.X_noTV.Data), squeeze(out.Y_noTV.Data), 'r--', 'LineWidth', 2); hold on;
plot(squeeze(out.X_TV.Data), squeeze(out.Y_TV.Data), 'b-', 'LineWidth', 2);
legend('Senza TV', 'Con TV');
xlabel('Posizione X [m]'); ylabel('Posizione Y [m]');
title('Confronto Traiettoria Step Steer');
axis equal; % Fondamentale per non distorcere la curva!
grid on;