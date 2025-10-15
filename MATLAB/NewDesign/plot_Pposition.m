function plot_Pposition(time_v,rP_v,fig_num)

figure(fig_num), clf;
plot(time_v,rP_v(1,:),'-','Color','#4DBEEE'); grid on; hold on;
plot(time_v,rP_v(2,:),'-','Color','#7E2F8E');
xlabel('Time (s)'); ylabel('Position (m)'); %title('Position of P'); 
legend('$P_x$','$P_y$','interpreter','latex');

end