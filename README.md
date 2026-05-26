# Controllo e Simulazione di una Pompa e Sistema a Doppio Serbatoio in MATLAB & Simulink

Questo repository contiene il materiale e i codici MATLAB/Simulink per la simulazione e il controllo di un sistema di serbatoi idraulici accoppiati (doppio serbatoio) alimentati da pompe, oltre a un esempio didattico di controllo predittivo (Model Predictive Control - MPC).

Il progetto è strutturato all'interno della cartella [`pompa_controllo_e_simulazione`](file:///Users/leonardoleggeri/Desktop/Pompa_controllo_matlab/pompa_controllo_e_simulazione).

---

## Indice
1. [Descrizione del Sistema a Doppio Serbatoio](#1-descrizione-del-sistema-a-doppio-serbatoio)
2. [Linearizzazione e Modello in Spazio di Stato](#2-linearizzazione-e-modello-in-spazio-di-stato)
3. [Progettazione dei Disaccoppiatori](#3-progettazione-dei-disaccoppiatori)
4. [Controllo Predittivo (MPC)](#4-controllo-predittivo-mpc)
5. [Struttura dei File](#5-struttura-dei-file)
6. [Requisiti e Utilizzo](#6-requisiti-e-utilizzo)

---

## 1. Descrizione del Sistema a Doppio Serbatoio

Il sistema idraulico è composto da due serbatoi collegati in cascata o in parallelo con interazioni reciproche:
* **Serbatoio 1 ($h_1$):** Il livello dell'acqua è regolato dall'azione di ingresso della pompa 1 e influenzato dalla pompa 2.
* **Serbatoio 2 ($h_2$):** Presenta uno scarico sul fondo caratterizzato da un coefficiente di efflusso $k$ ed è alimentato dalla pompa 2.

### Equazioni Dinamiche Non Lineari
Le equazioni differenziali che descrivono i livelli d'acqua $h_1$ e $h_2$ nei serbatoi sono:
$$\dot{h}_1(t) = u_1(t) - u_2(t)$$
$$\dot{h}_2(t) = u_2(t) - k\sqrt{h_2(t)}$$

Dove:
* $h_1, h_2$ sono i livelli dei serbatoi (in metri).
* $u_1, u_2$ sono le portate fornite dalle pompe (ingressi di controllo).
* $k$ è il coefficiente di scarico (impostato a $0.05$).

---

## 2. Linearizzazione e Modello in Spazio di Stato

Per progettare i regolatori lineari, il sistema viene linearizzato attorno a un punto di equilibrio definito dai livelli di riferimento desiderati:
* $h_{1, ref} = 1.0\text{ m}$
* $h_{2, ref} = 1.2\text{ m}$

All'equilibrio, gli ingressi costanti di mantenimento sono pari a:

$$u_{1, ref} = u_{2, ref} = k\sqrt{h_{2, ref}}$$

Definendo le variabili di deviazione rispetto al punto di equilibrio $\tilde{x} = x - x_{ref}$ e $\tilde{u} = u - u_{ref}$, si ottiene il modello nello spazio di stato continuo $\dot{\tilde{x}} = A\tilde{x} + B\tilde{u}$:

$$A = \begin{bmatrix} 0 & 0 \\ 0 & -\frac{k}{2\sqrt{h_{2, ref}}} \end{bmatrix}, \quad B = \begin{bmatrix} 1 & -1 \\ 0 & 1 \end{bmatrix}$$

$$C = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}, \quad D = \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$$

Il sistema continuo viene quindi discretizzato con un tempo di campionamento $T_s = 0.1\text{ s}$ per l'implementazione digitale del controllo.
---

## 3. Progettazione dei Disaccoppiatori

Il sistema presenta un forte accoppiamento incrociato (MIMO): la pompa 2 influenza contemporaneamente sia il livello $h_1$ sia il livello $h_2$. Per poter controllare in modo indipendente i due livelli con regolatori SISO separati ($C_1$ e $C_2$), viene progettata una matrice di precompensazione (disaccoppiatore) $M(z)$ tale che la funzione di trasferimento complessiva vista dai controllori sia diagonale.

Data la matrice di trasferimento del sistema $G(z)$:
$$G(z) = \begin{bmatrix} G_{11}(z) & G_{12}(z) \\ G_{21}(z) & G_{22}(z) \end{bmatrix}$$

La matrice di disaccoppiamento semplificata ha la forma:
$$M(z) = \begin{bmatrix} 1 & M_{12}(z) \\ 0 & 1 \end{bmatrix}$$
con:
$$M_{12}(z) = -G_{12}(z) \cdot G_{22}^{-1}(z)$$

Questo cancella l'effetto dell'ingresso $v_2$ (uscita del regolatore $C_2$) sulla dinamica del primo serbatoio.

---

## 4. Controllo Predittivo (MPC)

Nel file [`EsempioMPC.m`](file:///Users/leonardoleggeri/Desktop/Pompa_controllo_matlab/pompa_controllo_e_simulazione/EsempioMPC.m) viene implementato un esempio dettagliato di regolatore MPC lineare quadratico (LQR a orizzonte finito) applicato a un sistema a tempo discreto bidimensionale.

L'obiettivo è minimizzare il seguente indice di costo a orizzonte $N=5$:
$$J = x(N)^T S x(N) + \sum_{k=0}^{N-1} \left( x(k)^T Q x(k) + u(k)^T R u(k) \right)$$

Vengono implementate e confrontate tre metodologie risolutive:
1. **Anello Chiuso (Programmazione Dinamica):** Risoluzione all'indietro nel tempo tramite l'equazione di Riccati per trovare la sequenza di guadagni di feedback $K_j$.
2. **Anello Aperto (Matrici di Predizione):** Formulazione vettoriale compatta per calcolare l'ottimo globale analitico in un solo passo tramite l'inversione della matrice Hessiana: $U = -H^{-1}f$.
3. **Programmazione Quadratica (Solver QP):** Risoluzione del problema tramite l'algoritmo di ottimizzazione numerica `quadprog` di MATLAB, che consente l'estensione immediata a problemi con vincoli.

Il codice verifica infine che le tre soluzioni coincidano numericamente a meno di tolleranze infinitesime.

---

## 5. Struttura dei File

La cartella principale contiene i seguenti file:

* **[`Exe1_CAM_main.m`](file:///Users/leonardoleggeri/Desktop/Pompa_controllo_matlab/pompa_controllo_e_simulazione/Exe1_CAM_main.m):** Script principale MATLAB. Inizializza i parametri idraulici, definisce il punto di equilibrio, calcola le matrici dello spazio di stato, genera i disaccoppiatori discreti, esegue la simulazione in Simulink e traccia i grafici di risposta temporale e degli ingressi (con e senza saturazione a $0.9$).
* **[`Exe1_CAM.slx`](file:///Users/leonardoleggeri/Desktop/Pompa_controllo_matlab/pompa_controllo_e_simulazione/Exe1_CAM.slx):** Schema a blocchi Simulink che modella la fisica del sistema a doppio serbatoio (sia lineare che non lineare), il blocco dei limiti delle pompe ($0 \le u_i \le 0.9$), il disaccoppiatore $M$ e i controllori di livello.
* **[`EsempioMPC.m`](file:///Users/leonardoleggeri/Desktop/Pompa_controllo_matlab/pompa_controllo_e_simulazione/EsempioMPC.m):** Script didattico autoportante sull'implementazione di algoritmi di controllo predittivo ad orizzonte finito.
* **`Esercitazione 1 - Disaccoppiatori.pdf`:** Dispense didattiche con la spiegazione teorica dei sistemi MIMO, accoppiamento idraulico e sintesi dei compensatori.
* **`Exe 1 matlab.docx`:** Relazione/traccia dell'esercitazione con i dettagli di implementazione del codice.

---

## 6. Requisiti e Utilizzo

### Requisiti
Per eseguire le simulazioni è necessario disporre di:
* MATLAB (versione R2020a o successiva consigliata)
* Simulink
* Control System Toolbox
* Optimization Toolbox (per l'utilizzo del solver `quadprog` in MPC)

### Come Eseguire la Simulazione del Serbatoio
1. Aprire MATLAB e posizionarsi nella cartella `pompa_controllo_e_simulazione`.
2. Eseguire lo script `Exe1_CAM_main.m`.
3. Lo script configurerà automaticamente i parametri nel workspace, avvierà il modello Simulink `Exe1_CAM.slx` e genererà i grafici con l'andamento dei livelli dei serbatoi e l'azione di controllo calcolata rispetto a quella saturata (bounded).
4. È possibile modificare i guadagni dei controllori `C1` e `C2` nel file `Exe1_CAM_main.m` (sezioni 6 e 7) per osservare come cambia la risposta del sistema.

### Come Eseguire l'Esempio MPC
1. Aprire ed eseguire lo script `EsempioMPC.m`.
2. Verranno stampati nella Command Window di MATLAB i vettori degli ingressi ottimi calcolati con i 3 metodi e la loro differenza numerica.
