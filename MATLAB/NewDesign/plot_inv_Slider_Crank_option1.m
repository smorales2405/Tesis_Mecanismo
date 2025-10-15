function plot_inv_Slider_Crank_option1(rA,rB,rC,rBS,rL1,rL2,rD,rE,rP,rP_v,c_current,c_min,c_max,s,t, LinkColor)
% plot_inv_Slider_Crank_option1
% Plots the five-bar mechanism with Option 1 configuration
% DE is vertical, EP rotates around E
      
    % Figure configuration
    plot(nan), xlabel('x (m)'), ylabel('y (m)');
    title({'Five-Bar Inverted Slider-Crank - Option 1 (Vertical DE)'});
    
    % Add point labels
    text(rA(1), rA(2)+s,'A','HorizontalAlignment','center'); 
    text(rD(1), rD(2)+s,'D','HorizontalAlignment','center');
    grid on; hold on;
    
    % Plot ground/frame
    ground_width = abs(rD(1) - rA(1));
    rectangle('Position', [rA(1)-1, rA(2)-0.5, ground_width+2, 0.5], ...
              'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k', 'LineWidth', 1);
    
    % Plot links
    % Link AB (Crank)
    plot([rA(1) rB(1)],[rA(2) rB(2)],'Color',LinkColor, 'LineWidth', 3);
    text(rB(1), rB(2)+s,'B','HorizontalAlignment','center');
    
    % Link BC (Coupler)
    plot([rB(1) rC(1)],[rB(2) rC(2)],'Color',LinkColor, 'LineWidth', 2.5);
    text(rC(1), rC(2)+s,'C','HorizontalAlignment','center');
    
    % Link DE (Vertical Actuator) - Show as a cylinder
    % Draw actuator cylinder
    cyl_width = 0.8;
    % Outer cylinder (fixed part)
    rectangle('Position', [rD(1)-cyl_width/2, rD(2), cyl_width, c_max], ...
              'EdgeColor', 'k', 'LineWidth', 2);
    % Inner cylinder (moving part)
    rectangle('Position', [rD(1)-cyl_width/2+0.1, rD(2), cyl_width-0.2, c_current], ...
              'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'k', 'LineWidth', 1);
    
    % Mark actuator limits
    plot([rD(1)-cyl_width rD(1)+cyl_width], [rD(2)+c_min rD(2)+c_min], 'k--', 'LineWidth', 1);
    plot([rD(1)-cyl_width rD(1)+cyl_width], [rD(2)+c_max rD(2)+c_max], 'k--', 'LineWidth', 1);
    text(rD(1)+cyl_width+0.5, rD(2)+c_min, 'MIN', 'FontSize', 8);
    text(rD(1)+cyl_width+0.5, rD(2)+c_max, 'MAX', 'FontSize', 8);
    
    text(rE(1), rE(2)+s,'E','HorizontalAlignment','center');
    
    % Link EP (Platform)
    plot([rE(1) rP(1)],[rE(2) rP(2)],'Color',[0.2 0.6 0.2], 'LineWidth', 3);
    text(rP(1), rP(2)+s,'P','HorizontalAlignment','center');
    
    % Plot slide base at C
    plot([rB(1) rBS(1)],[rB(2) rBS(2)],'Color',LinkColor, 'LineWidth', 2);
    plot([rBS(1) rL1(1)],[rBS(2) rL1(2)],'Color',LinkColor, 'LineWidth', 2);
    plot([rBS(1) rL2(1)],[rBS(2) rL2(2)],'Color',LinkColor, 'LineWidth', 2);
    
    % Draw sliding connection between C and EP - PERPENDICULAR CONNECTION
    % Show that BC is perpendicular to PE
    plot([rC(1) rE(1)], [rC(2) rE(2)], 'g--', 'LineWidth', 1, 'Color', [0.2 0.6 0.2]);
    
    % % Add perpendicular symbol at C
    % % Calculate perpendicular symbol
    % vec_BC = [rC(1) - rB(1); rC(2) - rB(2)];
    % vec_EC = [rC(1) - rE(1); rC(2) - rE(2)];
    % vec_BC = vec_BC / norm(vec_BC);
    % vec_EC = vec_EC / norm(vec_EC);
    % 
    % % Draw small square to indicate 90° angle
    % square_size = 0.3;
    % square_corner = rC - square_size * vec_BC - square_size * vec_EC;
    % square_pts = [square_corner, square_corner + square_size*vec_BC, ...
    %               rC, square_corner + square_size*vec_EC, square_corner];
    % plot(square_pts(1,:), square_pts(2,:), 'k-', 'LineWidth', 1);
           
    % Plot joints on linkage
    plot([rA(1) rD(1)],...
         [rA(2) rD(2)],...
         "^",'MarkerSize',6,'MarkerFaceColor',LinkColor,'Color',LinkColor);
 
    % Plot joints on linkage
    plot([rB(1) rE(1) rP(1)],...
         [rB(2) rE(2) rP(2)],...
         'o','MarkerSize',4,'MarkerFaceColor',LinkColor,'Color',LinkColor);

    % Highlight sliding joint at C
    %plot(rC(1), rC(2), 'square', 'MarkerSize', 4, ...
    %     'MarkerFaceColor', [0.8 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 2);
     
    % Trajectory of P
    plot(rP_v(1,:),rP_v(2,:),'.','Color','#D95319', 'MarkerSize', 3); 
    
    % Add direction arrows for moving parts
    if ~isempty(rP_v)
        % Arrow showing actuator motion
        if c_current < c_max && c_current > c_min
            quiver(rE(1)+1, rE(2), 0, 0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        end
    end

end