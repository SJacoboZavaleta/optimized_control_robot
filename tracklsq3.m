function F = tracklsqmin(pid,y)

Kp = pid(1);
Ki = pid(2);
Kd = pid(3);

opt = simset('solver','ode8','SrcWorkspace','Current');
set_param('tf2/Controller','P',num2str(Kp));
set_param('tf2/Controller','I',num2str(Ki));
set_param('tf2/Controller','D',num2str(Kd));
[tout,xout,yout] = sim('tf2',[0 20]);

drawnow;

F=sum(abs(yout-1));