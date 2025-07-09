num = [0.99867];
den = [4.0786 3.4057 3.6303 1];
sys = tf(num,den);

[sysb,sigma] = balreal(sys);  
sysr = balred(sysb,2);

[num1, den2] = ss2tf(sysr.A,sysr.B, sysr.C, sysr.D);

systf = tf(num1, den2);
systf

step(sys,systf)

[C, info] = pidtune(systf, 'PID', 1)

