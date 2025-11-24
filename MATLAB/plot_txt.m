% Nombre del archivo (ajústalo si usas otro)
filename = 'datos_esp32.txt';

% Leer los datos (asume que son solo números separados por espacios)
data = readmatrix(filename);

% Extraer columnas
O_y = data(:, 1);   % Primera columna
th4 = data(:, 2);   % Segunda columna

rows = size(O_y,1);
dt = 0.1;
t2 = 0:dt:rows*dt-dt;

figure(1), clf;
plot(t2, O_y,'-','Color',"#8516D1");
ylabel('Posición (cm)'); title('Posición de O (ESP32)'); 
xlabel('Tiempo (s)');
xlim([0 10]);
%legend('$P_x$','$P_y$','interpreter','latex'); 
%ylim([0 15]); grid on;

figure(2), clf;
plot(t2,th4,'Color',"#DD5400"); grid on;
ylabel('Ángulo (deg)'); title('Angulo $\theta_4$ (ESP32)', 'interpreter','latex');
xlabel('Tiempo (s)');
xlim([0 10]); %ylim([-15 15])
