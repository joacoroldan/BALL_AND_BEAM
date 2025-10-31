clc; clear;
%{
    Se deben obtener ciertos parametros del sistema, principalmente del motor, del balin y las ganancias de los sensores
%}
k       = 20;
a       =  3;
A_th    =  1;
A_x     =  1;
rho     =  7;

%{
alpha y Kv son ganancias del controlador que debe ser diseñado y dependen
del modelo físico al ser implementado, ya que se realizara por medio de
simulación no tendremos mayor problema con esto, pero es importante tenerlo
en cuenta al trabajar con un modelo físico
alpha tiene como efecto a afectar la velocidad de resuesta del sistema
Kv ayuda a disminuir las oscilaciones
Tomando esto en cuenta, se realiza la simulacion del siguiente sistema,
proponiendo valores para alpha y Kv
%}
alpha   = 20;
Kv      =  0.65;

motor   = tf(k, [1 a 0]);
rv      = A_th * Kv * tf([1 0], 1);
g1      = feedback(motor, rv, -1);   
g2      = feedback(alpha * g1, A_th, -1);

% Grafica de la respuesta de la barra al escalon
figure(1)
step(g2);
grid on
hold on
titulo = ['Respuesta de la barra al escalon con (\alpha = ', num2str(alpha),' Kv =', num2str(Kv) ,')'];
title(titulo, 'Interpreter', 'tex', 'FontSize', 10);

%{
Se procede a definir la dinamica del balin, y se agrega al sistema de la
barra
%}
balin = tf(rho, [1 0 0]);
gh = g2 * balin;

[num, den] = tfdata(gh,'v');
syms s w real positive
num_s = poly2sym(num, s);
den_s = poly2sym(den, s);

GH_s = num_s / den_s;
GH_jw = subs(GH_s, s, 1j*w);
equation = abs(GH_jw) == 1;
w_sol = solve(equation, w);
w_gc = double(w_sol);

angle_rad = angle(GH_jw);
f_w_gc = subs(angle_rad, w, w_gc);
g_f_w_gc = rad2deg(double(f_w_gc)) - 360;

figure(2)
bode(gh);
grid on;
set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1);
all_axes = findobj(gcf, 'Type', 'axes');
mag_ax = all_axes(2); 
fase_ax = all_axes(1);
axes(mag_ax);
hold on;
xline(w_gc,'r-',['\omega_{gc}=', num2str(w_gc), '                        ']);
yline(0,'r');
plot(w_gc, 0, 'o')
axes(fase_ax);
xline(w_gc,'r-',['\omega_{gc}=', num2str(w_gc), '            ']);
yline(g_f_w_gc, 'r', ['            M_{f}=', num2str(g_f_w_gc)]);
plot(w_gc, g_f_w_gc, 'o');
titulo = 'Grafica de Bode de GH(s)';
title(titulo, 'Interpreter', 'tex', 'FontSize', 10);

%{
Dado que el ejercicio pide proponer los valores tanto para la frecuencia de
cruce de ganancia como para el margen de fase, se hace la propuesta
basandola en lo siguiente
Dado que el margen de fase esta relacionado con el amortiguamiento, y este
a su vez esta relacionado con el sobrepaso, se optara por proponer un
sobrepaso, para apartir de este calcular el amortiguamiento y el margen de
fase
Y se propone la frecuencia de cruce de ganancia
%}
w_cg = 2;
Mp = 25;
zetha = sqrt((log(Mp/100)^2)/((log(Mp/100)^2)+ pi^2));
%{
En clase se resalto la siguiente relación entre zetha y el margen de fase, por lo que
%}

m_fase = zetha * 100;
fase_r = 180 - m_fase;



fase = -1 *g_f_w_gc - fase_r;

%{
Para encontrar los valores del compezador, se realizara el procedimiento
visto en clase, que consiste en simular el compensador buscando que se de
el adelanto que se necesita
%}

xi = (1-sind(fase))/(1+sind(fase));
gd = tf([1 1], [1 1/xi]);

figure(3)
bode(gd)
grid on;
set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1);
all_axes = findobj(gcf, 'Type', 'axes');
mag_ax = all_axes(2); 
fase_ax = all_axes(1);
axes(mag_ax);
hold on;
xline(2.51, 'r');
yline(-7.99, 'r');
plot(2.51, -7.99, 'o');
axes(fase_ax);
xline(2.51, 'r');
yline(fase, 'r');
plot(2.51, fase, 'o');


gamma = 10^((7.99-4.95)/20);

T = 2.51 / w_cg;
b = 1/T;
c = (1/xi)*(1/T);

compensator = gamma * tf([1 b], [1 c]);
M = feedback(compensator*gh, 1, -1);



figure(4)
bode(compensator*gh)
grid on;
set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1);
all_axes = findobj(gcf, 'Type', 'axes');
mag_ax = all_axes(2); 
fase_ax = all_axes(1);
axes(mag_ax);
hold on;
xline(2, 'r');
yline(0, 'r');
plot(2, 0, 'o');
axes(fase_ax);
xline(2, 'r');
yline(-1 * fase_r, 'r');
plot(2, -1 * fase_r, 'o');

figure(5)
step(M)
grid on;