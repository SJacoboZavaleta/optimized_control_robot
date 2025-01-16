VTR = 1.e-6; 
D = 3; 
XVmin = [0,0,0]; 
XVmax = [5,1,1];
y=[0,0,0]; 
NP = 5; 
itermax = 100; 
F = 0.8; 
CR = 0.8; 
strategy = 7;
refresh = 5; 
        
tf2

%pid0 = [1.1470   31.0626    0.0003];
[x,f,nf] = devec3('tracklsq3',VTR,D,XVmin,XVmax,y,NP,itermax,F,CR,strategy,refresh);
Kp = x(1);
Ki = x(2);
Kd = x(3);

Gc = tf([Kd, Kp, Ki],[1,0]);


