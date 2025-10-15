function plot_inv_Slider_Crank_mod(rA,rB,rC,rBS,rL1,rL2,rD,rE,rP,rP_v,s,t, LinkColor)
      
    %Figure configuration
    plot(nan), xlabel('x (m)'), ylabel('y (m)');
    title({'Five-Bar Inverted Slider-Crank Mechanism'});%,['Time: ',num2str(t),' seconds']});
    text(rA(1), rA(2)+s,'A','HorizontalAlignment','center'); 
    text(rD(1), rD(2)+s,'D','HorizontalAlignment','center');
    grid on; hold on;
    
    %Plot links
    plot([rA(1) rB(1)],[rA(2) rB(2)],'Color',LinkColor);
    text(rB(1), rB(2)+s,'B','HorizontalAlignment','center');
    plot([rB(1) rC(1)],[rB(2) rC(2)],'Color',LinkColor);
    text(rC(1), rC(2)+s,'C','HorizontalAlignment','center');
    plot([rD(1) rE(1)],[rD(2) rE(2)],'Color',LinkColor);
    text(rE(1), rE(2)+s,'E','HorizontalAlignment','center');
    plot([rE(1) rP(1)],[rE(2) rP(2)],'Color',LinkColor);
    text(rP(1), rP(2)+s,'P','HorizontalAlignment','center');
    %Plot slidebase
    plot([rB(1) rBS(1)],[rB(2) rBS(2)],'Color',LinkColor);
    plot([rBS(1) rL1(1)],[rBS(2) rL1(2)],'Color',LinkColor);
    plot([rBS(1) rL2(1)],[rBS(2) rL2(2)],'Color',LinkColor);
           
    % plot joints on linkage
    plot([rA(1) rB(1) rD(1) rP(1)],...
    [rA(2) rB(2) rD(2) rP(2)],...
    'o','MarkerSize',5,'MarkerFaceColor',LinkColor,'Color',LinkColor);
     
    %Trajectory of P
    plot(rP_v(1,:),rP_v(2,:),'.','Color','#D95319'); 

end