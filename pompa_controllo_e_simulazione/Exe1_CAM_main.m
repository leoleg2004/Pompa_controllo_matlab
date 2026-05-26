%% 1. Parametri del doppio serbatoio
% Dati presi dalla presentazione - Esercitazione 1 - Disaccoppiatori.pdf

% Coefficiente di scarico 
k = 0.05;

% Condizione iniziale
h1_ini = 0.7; % m
h2_ini = 0.6; % m (metri)

% Livello di riferimento (desiderato)
h1_ref = 1;
h2_ref = 1.2;

% Tempo di campionamento
Ts = 0.1; % s

% Tempo di simulazione
T_sim = 20; 

% Vincoli di ingresso
u1_min = 0;
u2_min = 0;

u1_max = 0.9;
u2_max = 0.9;

% Punto di equilibrio - ingressi
u1_ref = k*sqrt(h2_ref);
u2_ref = u1_ref;

%% 2. Creazione del modello linearizzato (state space)
A = [0 0; 0 -k/(2*sqrt(h2_ref))];
B = [1 -1; 0 1];
C = eye(2); %crea una matrice identità 2x2
D = zeros(2,2); %crea una matrice di zeri, dandogli in ingresso come paramtri il numero di righe e colonne

%% 3. Calcolo della Fdt a tempo continuo e a tempo discreto 
s = tf('s'); 

G_ct = C*(s*eye(2) - A) \ B; % \ calcola l'inverso di quello che ha prima (s*eye(2) - A)

%converto a tempo discreto
G = c2d(G_ct, Ts);

%% 4. Progettazione dei disaccoppiatori
M12 = -G(1,2)*G(2,2)^(-1);
M11 = 1;
M21 = 0;
M22 = M11;

M = [M11 M12; M21 M22];

%% 5. Modello in simulink
model = 'Exe1_CAM.slx';

%% 6. Configurazione 1 controllore
C1 = 1;
C2 = 1;

% Simulazione del sistema 
output_1 = sim(model);

% Plot dei risultati
figure
plot(output_1.h1_log, 'DisplayName','$h_1$ [m]')
hold on;
plot(output_1.h2_log, 'DisplayName','$h_2$ [m]');
yline(h1_ref, 'Label', '$h_{1, ref}$', HandleVisibility='off');
yline(h2_ref, 'Label', '$h_{2, ref}$', HandleVisibility='off');
grid on;
xlabel('Time [s]');
ylabel('Height [m]');
title('Tank levels over time');
legend show;
grid on;

figure

subplot(2, 1, 1)
stairs(output_1.u1_log_unbound.Time, output_1.u1_log_unbound.Data, 'DisplayName', 'Computed control action')
hold on;
stairs(output_1.u1_log_bound.Time, output_1.u1_log_bound.Data, 'DisplayName', 'Bounded control action')
title('Azione di controllo pompa 1')
grid on;
legend show;


subplot(2, 1, 2)
stairs(output_1.u2_log_unbound.Time, output_1.u2_log_unbound.Data, 'DisplayName', 'Computed control action')
hold on;
stairs(output_1.u2_log_bound.Time, output_1.u2_log_bound.Data, 'DisplayName', 'Bounded control action')
title('Azione di controllo pompa 2')
grid on;
legend show;

% Computed control action: è l'azione di controllo ideale calcolata dai controllori
% C1, C2 e dal disaccoppiatore M. È ciò che il regolatore vorrebbe applicare per raggiungere 
% il riferimento nel minor tempo possibile. 
% Come vedi nel primo grafico della pompa 1, all'istante $t=1s$, il calcolo matematico richiede un picco che supera 
% il valore di $0.9$. 

% Bounded control action: È l'azione di controllo reale che entra nel
% sistema fisico che ha imposto limiti sull'azione di controllo di
% erogazione della pompa

%% 7. Configurazione 2 controllore
C1 = 5;
C2 = 1;

% Simulazione del sistema 
output_1 = sim(model);

% Plot dei risultati
figure
plot(output_1.h1_log, 'DisplayName','$h_1$ [m]')
hold on;
plot(output_1.h2_log, 'DisplayName','$h_2$ [m]');
yline(h1_ref, 'Label', '$h_{1, ref}$', HandleVisibility='off');
yline(h2_ref, 'Label', '$h_{2, ref}$', HandleVisibility='off');
grid on;
xlabel('Time [s]');
ylabel('Height [m]');
title('Tank levels over time');
legend show;
grid on;

figure

subplot(2, 1, 1)
stairs(output_1.u1_log_unbound.Time, output_1.u1_log_unbound.Data, 'DisplayName', 'Computed control action')
hold on;
stairs(output_1.u1_log_bound.Time, output_1.u1_log_bound.Data, 'DisplayName', 'Bounded control action')
title('Azione di controllo pompa 1')
grid on;
legend show;


subplot(2, 1, 2)
stairs(output_1.u2_log_unbound.Time, output_1.u2_log_unbound.Data, 'DisplayName', 'Computed control action')
hold on;
stairs(output_1.u2_log_bound.Time, output_1.u2_log_bound.Data, 'DisplayName', 'Bounded control action')
title('Azione di controllo pompa 2')
grid on;
legend show;