function plot_five_link_inv_Slider_Crank(rA,rB,rC,rD,rE,rMidPE,rP,rMidPE_v,c_current,c_min,c_max,s,t)
% plot_inv_Slider_Crank_option1
% Plots the five-bar mechanism with Option 1 configuration
% DE is vertical telescopic actuator, EP rotates around E
      
    % Figure configuration
    plot(nan), xlabel('x (m)'), ylabel('y (m)');
    title({'Banco de pruebas oscilante'});
    
    %% Add point labels
    text(rA(1), rA(2)-1.5*s,'A','HorizontalAlignment','center'); 
    text(rD(1), rD(2)-1.5*s,'D','HorizontalAlignment','center');
    grid on; hold on;
    
    % Plot ground/frame
    ground_width = abs(rD(1) - rA(1));
    rectangle('Position', [rA(1)-1, rA(2)-0.5, ground_width+2, 0.5], ...
              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k', 'LineWidth', 1);
    
    %% Plot links as rectangular bars
    % BARRA AB (Crank) - Rectangular gris
    bar_width_AB = 0.25;  % Ancho de la barra AB
    angle_AB = atan2(rB(2)-rA(2), rB(1)-rA(1));
    e_AB = [cos(angle_AB); sin(angle_AB)];
    e_perp_AB = [-sin(angle_AB); cos(angle_AB)];
    length_AB = norm([rB(1)-rA(1); rB(2)-rA(2)]);
    
    corner1_AB = rA - (bar_width_AB/2)*e_perp_AB;
    corner2_AB = rA + length_AB*e_AB - (bar_width_AB/2)*e_perp_AB;
    corner3_AB = rA + length_AB*e_AB + (bar_width_AB/2)*e_perp_AB;
    corner4_AB = rA + (bar_width_AB/2)*e_perp_AB;
    
    AB_x = [corner1_AB(1), corner2_AB(1), corner3_AB(1), corner4_AB(1), corner1_AB(1)];
    AB_y = [corner1_AB(2), corner2_AB(2), corner3_AB(2), corner4_AB(2), corner1_AB(2)];
    patch(AB_x, AB_y, [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    text(rB(1), rB(2)+s,'B','HorizontalAlignment','center');
    
    %% TELESCOPIC ACTUATOR DE (Similar to Inventor design)
    % Dimensions for the telescopic cylinder
    outer_cyl_width = 0.8;   % Width of outer cylinder (fixed part)
    inner_cyl_width = 0.5;   % Width of inner cylinder (moving part)
    base_height = 0.8;       % Height of the base mount
    
    % 1. Base mount at D (similar to the gray base in Inventor)
    rectangle('Position', [rD(1)-outer_cyl_width/2-0.2, rD(2)-base_height/2, ...
                           outer_cyl_width+0.4, base_height], ...
              'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Add mounting details
    plot([rD(1)-0.3 rD(1)+0.3], [rD(2) rD(2)], 'k-', 'LineWidth', 2);
    plot(rD(1), rD(2), 'o', 'MarkerSize', 8, ...
         'MarkerFaceColor', [0.4 0.4 0.4], 'MarkerEdgeColor', 'k');
    
    % 2. Outer cylinder (fixed part) - Dark gray/black
    % This represents the outer housing
    fixed_cylinder_height = c_min + 0.5; % Fixed part extends to minimum + buffer
    rectangle('Position', [rD(1)-outer_cyl_width/2, rD(2)+base_height/4, ...
                           outer_cyl_width, fixed_cylinder_height], ...
              'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Add detail lines on outer cylinder (like slots in Inventor design)
    num_slots = 8;
    slot_spacing = fixed_cylinder_height / num_slots;
    for i = 1:num_slots-1
        y_pos = rD(2) + base_height/4 + i*slot_spacing;
        plot([rD(1)-outer_cyl_width/2+0.05, rD(1)+outer_cyl_width/2-0.05], ...
             [y_pos, y_pos], 'Color', [0.5 0.5 0.5], 'LineWidth', 0.5);
    end
    
    % 3. Inner cylinder (moving part) - Red/orange color like in your image
    % Calculate how much extends beyond the fixed part
    extension = c_current - fixed_cylinder_height;
    if extension > 0
        % Draw the extending part
        rectangle('Position', [rD(1)-inner_cyl_width/2, ...
                               rD(2)+base_height/4+fixed_cylinder_height, ...
                               inner_cyl_width, extension], ...
                  'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1.5);
        
        % Add some detail lines on the inner cylinder
        if extension > 0.5
            num_lines = floor(extension/0.5);
            for i = 1:num_lines
                y_pos = rD(2) + base_height/4 + fixed_cylinder_height + i*0.5;
                if y_pos < rE(2) - 0.2
                    plot([rD(1)-inner_cyl_width/2+0.02, rD(1)+inner_cyl_width/2-0.02], ...
                         [y_pos, y_pos], 'Color', [0.6 0.1 0.1], 'LineWidth', 0.5);
                end
            end
        end
    end
    
    % 4. Top connector piece at E (similar to the white connector in Inventor)
    connector_width = 0.6;
    connector_height = 0.3;
    rectangle('Position', [rE(1)-connector_width/2, rE(2)-connector_height/2, ...
                           connector_width, connector_height], ...
              'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Joint at E
    plot(rE(1), rE(2), 'o', 'MarkerSize', 6, ...
         'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % Mark actuator limits with subtle lines
    plot([rD(1)+outer_cyl_width+0.3, rD(1)+outer_cyl_width+0.1], ...
         [rD(2)+c_min, rD(2)+c_min], 'k--', 'LineWidth', 0.5);
    plot([rD(1)+outer_cyl_width+0.3, rD(1)+outer_cyl_width+0.1], ...
         [rD(2)+c_max, rD(2)+c_max], 'k--', 'LineWidth', 0.5);
    text(rD(1)+outer_cyl_width+1, rD(2)+c_min, 'MIN', ...
         'FontSize', 7, 'HorizontalAlignment', 'right');
    text(rD(1)+outer_cyl_width+1, rD(2)+c_max, 'MAX', ...
         'FontSize', 7, 'HorizontalAlignment', 'right');
    
    % Draw a vertical dashed line showing current extension
    % plot([rD(1)+outer_cyl_width+0.2, rD(1)+outer_cyl_width+0.2], ...
    %      [rD(2), rE(2)], 'b:', 'LineWidth', 1);
    % text(rD(1)+outer_cyl_width+0.3, (rD(2)+rE(2))/2, ...
    %      sprintf('%.1fm', c_current), 'FontSize', 8, 'Color', 'b');
    
    %% Platform and connections
    text(rE(1), rE(2)+s,'E','HorizontalAlignment','center');
    
    % Calcular ángulo y vectores de la barra EP
    angle_EP = atan2(rP(2)-rE(2), rP(1)-rE(1));
    e_along = [cos(angle_EP); sin(angle_EP)];
    e_perp = [-sin(angle_EP); cos(angle_EP)];
    bar_length = norm([rP(1)-rE(1); rP(2)-rE(2)]);
    
    % Dimensiones del perfil EP
    main_bar_height = 0.35;    % Altura de la barra principal
    rail_height = 0.25;         % Altura del riel inferior
    rail_gap = 0.35;           % Separación entre barra principal y riel
    
    % 1. BARRA PRINCIPAL (gris claro) - parte superior
    % Posicionar desde E hacia P
    main_bottom_left = rE - (main_bar_height/2)*e_perp;
    corner1_main = main_bottom_left;
    corner2_main = main_bottom_left + bar_length*e_along;
    corner3_main = main_bottom_left + bar_length*e_along + main_bar_height*e_perp;
    corner4_main = main_bottom_left + main_bar_height*e_perp;
    
    main_x = [corner1_main(1), corner2_main(1), corner3_main(1), corner4_main(1), corner1_main(1)];
    main_y = [corner1_main(2), corner2_main(2), corner3_main(2), corner4_main(2), corner1_main(2)];
    patch(main_x, main_y, [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % 2. RIEL INFERIOR (gris medio) - FIJO en la barra PE
    % El riel es parte fija de la estructura PE (no se mueve con C)
    rail_length = 12.5;  % Longitud del riel en metros
    rail_start_distance = 10;  % Distancia desde E donde empieza el riel
    
    % El riel empieza a cierta distancia desde E
    rail_start_point = rE + rail_start_distance*e_along;
    
    % Posicionar el riel DEBAJO de la barra (invertir el sentido de e_perp)
    % La barra principal va desde main_bottom_left hacia arriba
    % El riel debe ir DEBAJO, así que restamos más
    rail_top_left = rail_start_point + (main_bar_height/2+rail_gap)*e_perp;
    
    % Esquinas del riel (construyendo hacia ABAJO desde rail_top_left)
    corner1_rail = rail_top_left;
    corner2_rail = rail_top_left + rail_length*e_along;
    corner3_rail = rail_top_left + rail_length*e_along - rail_height*e_perp;
    corner4_rail = rail_top_left - rail_height*e_perp;
    
    rail_x = [corner1_rail(1), corner2_rail(1), corner3_rail(1), corner4_rail(1), corner1_rail(1)];
    rail_y = [corner1_rail(2), corner2_rail(2), corner3_rail(2), corner4_rail(2), corner1_rail(2)];
    patch(rail_x, rail_y, [0.55 0.55 0.55], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Agregar ranuras/slots en el riel
    num_slots = floor(rail_length / 1.5);
    for i = 1:num_slots
        slot_pos = rail_start_point + (i * rail_length/(num_slots+1))*e_along;
        slot_top = slot_pos + (main_bar_height/2)*e_perp + rail_gap*e_perp;
        slot_bottom = slot_top - (rail_height - 0.06)*e_perp;
        plot([slot_top(1) slot_bottom(1)], [slot_top(2) slot_bottom(2)], ...
             'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
    end
    
    text(rP(1), rP(2)+s,'P','HorizontalAlignment','center');
    
    %% SLIDER en C sobre el riel inferior
    slider_length = 0.9;
    slider_height = rail_height+0.2;  % Mismo alto que el riel para que encaje perfectamente
    
    % Proyectar C sobre la línea EP para saber dónde está sobre la barra
    vec_EC = [rC(1) - rE(1); rC(2) - rE(2)];
    projection_length = dot(vec_EC, e_along);
    C_on_bar = rE + projection_length * e_along;
    
    % Calcular centro del slider - debe estar en el centro del riel
    % El riel empieza en: rail_top_left = punto + (main_bar_height/2+rail_gap)*e_perp
    % El centro del riel está a: rail_top_left - (rail_height/2)*e_perp
    % Entonces: centro = punto + (main_bar_height/2 + rail_gap - rail_height/2)*e_perp
    slider_center = C_on_bar + (main_bar_height/2 + rail_gap - rail_height/2)*e_perp;
    % Esquinas del slider
    corner1_slider = slider_center - (slider_length/2)*e_along - (slider_height/2)*e_perp;
    corner2_slider = slider_center + (slider_length/2)*e_along - (slider_height/2)*e_perp;
    corner3_slider = slider_center + (slider_length/2)*e_along + (slider_height/2)*e_perp;
    corner4_slider = slider_center - (slider_length/2)*e_along + (slider_height/2)*e_perp;
    
    slider_x = [corner1_slider(1), corner2_slider(1), corner3_slider(1), corner4_slider(1), corner1_slider(1)];
    slider_y = [corner1_slider(2), corner2_slider(2), corner3_slider(2), corner4_slider(2), corner1_slider(2)];
    patch(slider_x, slider_y, [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 2);
         
    %% BARRA BC (Coupler) - Rectangular gris
    bar_width_BC = 0.25;  % Ancho de la barra BC
    angle_BC = atan2(rC(2)-rB(2), rC(1)-rB(1));
    e_BC = [cos(angle_BC); sin(angle_BC)];
    e_perp_BC = [-sin(angle_BC); cos(angle_BC)];
    rC2 = slider_center + slider_height/2*e_perp;
    length_BC = norm([rC2(1)-rB(1); rC2(2)-rB(2)]);
    
    corner1_BC = rB - (bar_width_BC/2)*e_perp_BC;
    corner2_BC = rB + length_BC*e_BC - (bar_width_BC/2)*e_perp_BC;
    corner3_BC = rB + length_BC*e_BC + (bar_width_BC/2)*e_perp_BC;
    corner4_BC = rB + (bar_width_BC/2)*e_perp_BC;
    
    BC_x = [corner1_BC(1), corner2_BC(1), corner3_BC(1), corner4_BC(1), corner1_BC(1)];
    BC_y = [corner1_BC(2), corner2_BC(2), corner3_BC(2), corner4_BC(2), corner1_BC(2)];
    patch(BC_x, BC_y, [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    text(rC2(1)+s, rC2(2)-s/2,'C','HorizontalAlignment','center');
    %% Plot joints on linkage
        
    % CHUMACERA en punto A (fijo en el suelo)
    bearing_size_A = 0.4;  % Tamaño del rodamiento
    bearing_base_width_A = 0.8;
    bearing_base_height_A = 0.3;
    
    % Base de la chumacera A (gris)
    rectangle('Position', [rA(1)-bearing_base_width_A/2, rA(2)-bearing_base_height_A, ...
                           bearing_base_width_A, bearing_base_height_A], ...
              'FaceColor', [0.65 0.65 0.65], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Círculo exterior de la chumacera A (gris claro)
    rectangle('Position', [rA(1)-bearing_size_A, rA(2)-bearing_size_A, ...
                           2*bearing_size_A, 2*bearing_size_A], ...
              'Curvature', [1 1], 'FaceColor', [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Círculo interior (eje) en A (gris oscuro)
    inner_radius_A = bearing_size_A * 0.4;
    rectangle('Position', [rA(1)-inner_radius_A, rA(2)-inner_radius_A, ...
                           2*inner_radius_A, 2*inner_radius_A], ...
              'Curvature', [1 1], 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Tornillos de montaje en A
    bolt_offset_A = 0.5;
    bolt_radius_A = 0.08;
    % Tornillo izquierdo
    rectangle('Position', [rA(1)-bolt_offset_A-bolt_radius_A, rA(2)-bearing_base_height_A/2-bolt_radius_A, ...
                           2*bolt_radius_A, 2*bolt_radius_A], ...
              'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1);
    % Tornillo derecho
    rectangle('Position', [rA(1)+bolt_offset_A-bolt_radius_A, rA(2)-bearing_base_height_A/2-bolt_radius_A, ...
                           2*bolt_radius_A, 2*bolt_radius_A], ...
              'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1);
    
    % CHUMACERA en punto B (unión entre barras AB y BC)
    bearing_size_B = 0.35;
    bearing_base_width_B = 0.7;
    bearing_base_height_B = 0.25;
    
    % Base de la chumacera B (gris) - rectángulo simple
    rectangle('Position', [rB(1)-bearing_base_width_B/2, rB(2)-bearing_base_height_B/2, ...
                           bearing_base_width_B, bearing_base_height_B], ...
              'FaceColor', [0.65 0.65 0.65], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Círculo exterior de la chumacera B (gris claro)
    rectangle('Position', [rB(1)-bearing_size_B, rB(2)-bearing_size_B, ...
                           2*bearing_size_B, 2*bearing_size_B], ...
              'Curvature', [1 1], 'FaceColor', [0.75 0.75 0.75], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Círculo interior (eje) en B (gris oscuro)
    inner_radius_B = bearing_size_B * 0.4;
    rectangle('Position', [rB(1)-inner_radius_B, rB(2)-inner_radius_B, ...
                           2*inner_radius_B, 2*inner_radius_B], ...
              'Curvature', [1 1], 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Tornillos de montaje en B
    bolt_offset_B = 0.4;
    bolt_radius_B = 0.06;
    % Tornillo superior
    rectangle('Position', [rB(1)-bolt_radius_B, rB(2)+bolt_offset_B-bolt_radius_B, ...
                           2*bolt_radius_B, 2*bolt_radius_B], ...
              'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1);
    % Tornillo inferior
    rectangle('Position', [rB(1)-bolt_radius_B, rB(2)-bolt_offset_B-bolt_radius_B, ...
                           2*bolt_radius_B, 2*bolt_radius_B], ...
              'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1);
    
    %% Plot midpoint of PE

    % Joint at mPE
    plot(rMidPE(1), rMidPE(2), 'o', 'MarkerSize', 6, ...
         'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'LineWidth', 1.5);
    text(rMidPE(1), rMidPE(2)+s,'O','HorizontalAlignment','center');

    % Trajectory of midpoint of PE
    plot(rMidPE_v(1,:),rMidPE_v(2,:),'.','Color','#D95319', 'MarkerSize', 3); 
    
    % Add motion indicators
    % if ~isempty(rP_v) && length(rP_v(1,:)) > 1
    %     % Arrow showing actuator motion direction
    %     if abs(c_current - c_min) > 0.1 && abs(c_current - c_max) > 0.1
    %         % Get actuator velocity direction
    %         if length(rP_v(1,:)) > 10
    %             if rE(2) > mean(rP_v(2,end-10:end))
    %                 arrow_dir = 1;  % Extending
    %                 arrow_color = 'r';
    %             else
    %                 arrow_dir = -1; % Contracting
    %                 arrow_color = 'b';
    %             end
    % 
    %             % Draw arrow next to actuator
    %             arrow_base = [rD(1) + outer_cyl_width + 0.5, (rD(2) + rE(2))/2];
    %             quiver(arrow_base(1), arrow_base(2), 0, 0.5*arrow_dir, ...
    %                    arrow_color, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    %         end
    %     end
    % end

end