function [theta3, theta4] = calc_theta3_theta4_nopt(rB, rE, b, e, theta3_prev, theta4_prev)
% calc_theta3_theta4 - Versión analítica con continuidad
% 
% Añadido: theta3_prev, theta4_prev para mantener continuidad
% Si no se proporcionan valores previos, usar nargin < 6

% Verificar si tenemos valores previos
if nargin < 6
    theta3_prev = [];
    theta4_prev = [];
    has_previous = false;
else
    has_previous = ~isempty(theta3_prev) && ~isempty(theta4_prev);
end

% Posiciones actuales
Bx = rB(1); By = rB(2);
Ex = rE(1); Ey = rE(2);

% Vector de E a B
EBx = Bx - Ex;
EBy = By - Ey;
EB_dist = sqrt(EBx^2 + EBy^2);

% Verificar si hay solución posible
if EB_dist > e + b
    % B está muy lejos, usar aproximación
    theta4 = atan2(EBy, EBx);
    theta3 = theta4 + pi/2;
    return;
end

% SOLUCIÓN GEOMÉTRICA
% Teorema de Pitágoras para triángulo rectángulo EBC
EC_dist_sq = EB_dist^2 - b^2;

if EC_dist_sq < 0
    % Configuración singular - usar mejor aproximación
    if has_previous
        theta3 = theta3_prev;
        theta4 = theta4_prev;
    else
        theta4 = atan2(EBy, EBx);
        theta3 = theta4 + pi/2;
    end
    return;
end

EC_dist = sqrt(EC_dist_sq);

% Ángulo de EB respecto al eje x
angle_EB = atan2(EBy, EBx);

% Ángulo en E entre EC y EB
angle_at_E = acos(EC_dist / EB_dist);

% Dos posibles posiciones para theta4
theta4_opt1 = angle_EB - angle_at_E;
theta4_opt2 = angle_EB + angle_at_E;

% Para cada theta4, hay dos posibles theta3 (perpendiculares)
solutions = zeros(4, 2); % [theta3, theta4] para cada solución

% Solución 1: theta4_opt1, theta3 = theta4 + pi/2
solutions(1, :) = [theta4_opt1 + pi/2, theta4_opt1];

% Solución 2: theta4_opt1, theta3 = theta4 - pi/2
solutions(2, :) = [theta4_opt1 - pi/2, theta4_opt1];

% Solución 3: theta4_opt2, theta3 = theta4 + pi/2
solutions(3, :) = [theta4_opt2 + pi/2, theta4_opt2];

% Solución 4: theta4_opt2, theta3 = theta4 - pi/2
solutions(4, :) = [theta4_opt2 - pi/2, theta4_opt2];

% Verificar cuál solución es válida y elegir la mejor
valid_solutions = [];
errors = [];

for i = 1:4
    th3 = solutions(i, 1);
    th4 = solutions(i, 2);
    
    % Calcular posición de C
    C = rB + b * [cos(th3); sin(th3)];
    
    % Vector EC
    EC = C - rE;
    EC_norm = EC / norm(EC);
    
    % Dirección de PE
    PE_dir = [cos(th4); sin(th4)];
    
    % Verificar que C está en la línea PE (EC paralelo a PE_dir)
    alignment = abs(dot(EC_norm, PE_dir));
    
    % Verificar que BC es perpendicular a PE
    BC_dir = [cos(th3); sin(th3)];
    perpendicularity = abs(dot(BC_dir, PE_dir));
    
    % Solo considerar soluciones válidas
    if alignment > 0.99 && perpendicularity < 0.01
        valid_solutions = [valid_solutions; i];
        
        % Si tenemos estado previo, calcular error respecto al estado anterior
        if has_previous
            % Normalizar ángulos para comparación
            dth3 = angle_difference(th3, theta3_prev);
            dth4 = angle_difference(th4, theta4_prev);
            error = dth3^2 + dth4^2;
        else
            % Sin estado previo, preferir configuración donde C está cerca del centro de PE
            dist_EC = norm(EC);
            ideal_dist = e * 0.5;
            error = (dist_EC - ideal_dist)^2;
        end
        errors = [errors; error];
    end
end

% Seleccionar la mejor solución
if isempty(valid_solutions)
    % No hay solución válida exacta, buscar la más cercana
    best_alignment = 0;
    best_idx = 1;
    
    for i = 1:4
        th3 = solutions(i, 1);
        th4 = solutions(i, 2);
        
        C = rB + b * [cos(th3); sin(th3)];
        EC = C - rE;
        EC_norm = EC / norm(EC);
        PE_dir = [cos(th4); sin(th4)];
        alignment = abs(dot(EC_norm, PE_dir));
        
        if alignment > best_alignment
            best_alignment = alignment;
            best_idx = i;
        end
    end
    
    theta3 = solutions(best_idx, 1);
    theta4 = solutions(best_idx, 2);
else
    % Elegir la solución con menor error (más continua)
    [~, min_idx] = min(errors);
    sol_idx = valid_solutions(min_idx);
    theta3 = solutions(sol_idx, 1);
    theta4 = solutions(sol_idx, 2);
end

% Normalizar ángulos a [0, 2*pi]
theta3 = mod(theta3, 2*pi);
theta4 = mod(theta4, 2*pi);

end

function diff = angle_difference(angle1, angle2)
% Calcula la diferencia mínima entre dos ángulos
% considerando la periodicidad de 2*pi

diff = angle1 - angle2;
% Normalizar a [-pi, pi]
while diff > pi
    diff = diff - 2*pi;
end
while diff < -pi
    diff = diff + 2*pi;
end

end