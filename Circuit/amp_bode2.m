pkg load control;
close all,; clear all;

Cf = 1.8e-9;
Cg = 470e-6;
Rg = 2e3;
Rf = 520e3;


s  = tf('s');
Zf = 1/(1/Rf+1/(1/(Cf*s)));
Zg = Rg+1/(Cg*s);

H = 1+Zf/Zg

a = Cg*Cf*Rg*Rf;
b = Cg*Rg;
c = Cg*Rf;
d = Cf*Rf;

H2 = (a*s^2+s*(b+c+d)+1)/(a*s^2+s*(b+d)+1);