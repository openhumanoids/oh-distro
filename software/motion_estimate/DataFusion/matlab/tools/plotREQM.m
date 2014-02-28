function plotREQM(REQM, fignumber)

if (nargin <2)
    fignumber = 1;
end
    
figure(fignumber), clf
subplot(411),plot(REQM.lQb(:,2:4)),title('REQ DataF quaternion vector'), grid on
subplot(412),plot(REQM.a_l),title('REQ DataF local frame acceleration'), grid on
subplot(413),plot(REQM.V_l),title('REQ DataF local velocity'), grid on
subplot(414),plot(REQM.P_l),title('REQ DataF local position'), grid on

