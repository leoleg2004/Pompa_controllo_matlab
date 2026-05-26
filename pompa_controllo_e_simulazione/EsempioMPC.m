clc
clearvars
close all

%% --- PARAMETRI COMUNI ---
A = [1 1; 0 1];
B = [0; 1];
Q = eye(2);
R = 1;
N = 5;
S = 100 * eye(2);
x0 = [-3; 1];

%% --- 1. SOLUZIONE IN ANELLO CHIUSO (Programmazione Dinamica) ---
% Calcolo dei guadagni K tornando indietro nel tempo (Riccati)
P_cl = zeros(2, 2, N+1);
K_cl = zeros(1, 2, N);
P_cl(:,:,N+1) = S; 

for j = N:-1:1
    K_cl(:,:,j) = (R + B' * P_cl(:,:,j+1) * B) \ (B' * P_cl(:,:,j+1) * A);
    P_cl(:,:,j) = Q + A' * P_cl(:,:,j+1) * A - A' * P_cl(:,:,j+1) * B * K_cl(:,:,j);
end

% Simulazione della traiettoria
x_cl = zeros(2, N+1);
x_cl(:,1) = x0;
u_cl_final = zeros(N, 1);

for j = 1:N
    u_cl_final(j) = -K_cl(:,:,j) * x_cl(:,j);
    x_cl(:,j+1) = A * x_cl(:,j) + B * u_cl_final(j);
end

disp('--- Metodo 1: Anello Chiuso ---')
disp('Vettore u (cl):')
disp(u_cl_final)

%% --- 2. SOLUZIONE IN ANELLO APERTO (Matrici di Predizione) ---
% Costruiamo le matrici Cal_A e Cal_B per calcolare la risposta forzata
Cal_A = []; 
Cal_B = [];

for i = 1:N
    Cal_A = [Cal_A; A^i];
    riga_B = [];
    for j = 1:N
        if j <= i
            riga_B = [riga_B, A^(i-j)*B];
        else
            riga_B = [riga_B, zeros(2,1)];
        end
    end
    Cal_B = [Cal_B; riga_B];
end

% Definizione pesi estesi
Cal_Q = blkdiag(kron(eye(N-1), Q), S);
Cal_R = kron(eye(N), R);

% Calcolo matrici H e f per il funzionale di costo J
H = 2 * (Cal_B' * Cal_Q * Cal_B + Cal_R);
f = 2 * (x0' * Cal_A' * Cal_Q * Cal_B)';

% Soluzione analitica del minimo: grad(J) = 0 -> u = -H \ f
u_ol = -H \ f;

disp('--- Metodo 2: Anello Aperto ---')
disp('Vettore u (ol):')
disp(u_ol)

%% --- 3. SOLUZIONE TRAMITE QUADRATIC PROGRAMMING (Solver) ---
% Utilizziamo il solver 'quadprog' di MATLAB. 
% Nota: Usiamo le matrici H e f calcolate nella sezione precedente.
% In assenza di vincoli, quadprog usa un algoritmo di tipo "reflective Newton" 
% o "interior-point" per trovare il minimo globale.

options = optimoptions('quadprog', 'Display', 'off');
u_qp = quadprog(H, f, [], [], [], [], [], [], [], options);

disp('--- Metodo 3: Quadratic Programming ---')
disp('Vettore u (qp):')
disp(u_qp)

%% --- CONFRONTO FINALE ---
% Verifichiamo che la differenza tra i metodi sia trascurabile (errore numerico)
diff_1_2 = norm(u_cl_final - u_ol);
diff_2_3 = norm(u_ol - u_qp);

fprintf('Differenza Metodo 1 vs 2: %e\n', diff_1_2);
fprintf('Differenza Metodo 2 vs 3: %e\n', diff_2_3);