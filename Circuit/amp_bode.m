pkg load control;
close all;clear all;

s  = tf('s');

## High pass filter(in-amp input)
C1 = 1e-6;
R1 = 1e6;

H1 = R1/(R1+1/(C1*s));

## Instrumentation amp
RinG = 2e3;

H2 = 1+100e3/RinG;

## Second Gain stage
Cf = 2.2e-9;
Cg = 470e-6;
Rg = 1000;
Rf = 440000;

Zf = 1/(1/Rf+1/(1/(Cf*s)));
Zg = Rg+1/(Cg*s);

H3 = 1+Zf/Zg;


## Low pass filter
C2 = 1e-6;
R2 = 1e3;

H4 = 1/(C2*s)/(R2+1/(C2*s));

H = H1*H2*H3*H4;


[mag, f] = bodemag(H);
hold on
[mag1, f1] = bodemag(H1);
[mag3, f3] = bodemag(H3);
[mag4, f4] = bodemag(H4);
#[mag,w] = bodemag(H);

semilogx(f/(2*pi), 20*log10(mag));
hold on
grid on
semilogx(f1/(2*pi), 20*log10(mag1));
hold on
semilogx(f3/(2*pi), 20*log10(mag3));
hold on
semilogx(f4/(2*pi), 20*log10(mag4));

title("Gain of circuit");
xlabel("Frequency (Hz)");
ylabel("Gain (dB)");

#semilogx(w, 20*log10(mag));

a=1/(Cg*Cf*Rg*Rf);
x = 1/(Rf*Cf);
y = 1/(Rg*Cf);
z = 1/(Rg*Cg);

#hold on
#H1 = (((s*(s+(x+y+z))+a)))/(((s*(s+(x+z))+a)));
#bodemag(H1);


#w2=0.00170648*((293i)*x + (293i)*z + (1000i)*y + sqrt(((-1000*i)*y + (-293i)*x + (-293i)*z)^2 + 343396*a));

###w1=0.00170648i*((293)*x + (293)*z + (1000)*y + i*sqrt(((-1000*i)*y + (-293i)*x + (-293i)*z)^2 + 343396*a));

#hold on;
#peak = 20*log10((x+y+z)/(x+z));
#plot(abs(w1), peak-3, '*r');
#plot(abs(w2), peak-3, '*r');

#w1 = abs(w1)*180/pi
#w2 = abs(w2)*180/pi