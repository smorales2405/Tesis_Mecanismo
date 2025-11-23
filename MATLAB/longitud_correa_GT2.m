function L = longitud_correa_GT2(N1, N2, C)
% LONGITUD_CORREA_GT2 Calcula la longitud de una correa cerrada GT2
%
% Sintaxis: L = longitud_correa_GT2(N1, N2, C)
%
% Entradas:
%   N1 - Número de dientes de la polea 1 (mayor o igual a N2)
%   N2 - Número de dientes de la polea 2
%   C  - Distancia entre centros de las poleas (mm)
%
% Salida:
%   L  - Longitud total de la correa GT2 (mm)
%
% Especificaciones de la correa:
%   - Tipo: GT2
%   - Paso: 2 mm
%   - Ancho: 10 mm
%
% Ejemplo:
%   L = longitud_correa_GT2(20, 20, 100)  % Poleas iguales de 20 dientes
%   L = longitud_correa_GT2(40, 20, 150)  % Poleas diferentes

    % Constantes
    P = 2;  % Paso de la correa GT2 en mm
    
    % Validaciones
    if N1 < 0 || N2 < 0 || C <= 0
        error('Los valores deben ser positivos');
    end
    
    if N1 < N2
        % Intercambiar para que N1 sea siempre mayor o igual
        temp = N1;
        N1 = N2;
        N2 = temp;
    end
    
    % Cálculo de la longitud de la correa
    % Fórmula: L = 2*C + π*P*(N1+N2)/2 + P²*(N1-N2)²/(4*π*C)
    
    parte1 = 2 * C;                              % Longitud de los tramos rectos
    parte2 = pi * P * (N1 + N2) / 2;            % Longitud en contacto con las poleas
    parte3 = (P^2 * (N1 - N2)^2) / (4 * pi * C); % Corrección por diferencia de diámetros
    
    L = parte1 + parte2 + parte3;
    
    % Mostrar información adicional
    fprintf('\n--- Cálculo de Longitud de Correa GT2 (10mm ancho) ---\n');
    fprintf('Polea 1: %d dientes (Ø = %.2f mm)\n', N1, N1*P/pi);
    fprintf('Polea 2: %d dientes (Ø = %.2f mm)\n', N2, N2*P/pi);
    fprintf('Distancia entre centros: %.2f mm\n', C);
    fprintf('Longitud de correa calculada: %.2f mm\n', L);
    fprintf('Número aproximado de dientes: %.1f\n', L/P);
    fprintf('----------------------------------------------------\n\n');
    
end