function [] = myStopFcn2simObst(xActEst,yActEst,X_array,Y_array,rp1,rp2,obstaclesArray)

figure
plot(yActEst.signals.values(:,1),xActEst.signals.values(:,1),'b-')
hold on
plot(yActEst.signals.values(:,2),xActEst.signals.values(:,2),'b--')
legend('Actual Path','Estimated Path')
plot(Y_array,X_array,'*r')
grid


for ii = 1:length(X_array)
    my_circle(rp2,Y_array(ii),X_array(ii),1,1,'-r',1)
    my_circle(rp1,Y_array(ii),X_array(ii),1,1,'--r',1)
end
% obstacle Placements
temp =size(obstaclesArray);
noOfObst = temp(2);
for ii = 1:noOfObst
    my_circleFill(obstaclesArray(3,ii),obstaclesArray(2,ii),obstaclesArray(1,ii),1,1,'r')
end
vAxis = axis;

xlabel('y [m]')
ylabel('x [m]')
title('Red Stars: wayPoints, Red Disk: Obstacles')
axis equal
axis([vAxis(1)-2*max(rp1,rp2) vAxis(2)+2*max(rp1,rp2) vAxis(3)-2*max(rp1,rp2) vAxis(4)+2*max(rp1,rp2)])

hold off
