# TurtleBot3 Maze Explorer
**Corso:** Mobile Robotics  
**Studenti:** Giorgio Scarnera, Rocco Scolletta, Marta Silvestri  
 
## Descrizione del progetto
Questo progetto implementa un'architettura per l'esplorazione autonoma e la navigazione all'interno di un labirinto simulato tramite Gazebo Harmonic e ROS 2 (Jazzy).  
Il robot, un TurtleBot3 modello Waffle, ha l'obiettivo di mappare un labirinto per lui sconosciuto (e da noi definito) in totale autonomia e raccogliere (per prossimità) 9 chiavi numerate sequenzialmente (rappresentate da marker ArUco), sfruttando le logiche di navigazione e una memoria spaziale per ottimizzare i percorsi.
 
## Architettura del sistema
Il sistema si basa sull'interazione di diversi nodi ROS 2, coordinati tramite un file di lancio master (`main.launch.py`) e supervisionati da un nodo centrale basato su una Macchina a Stati Finita (FSM).
 
### 1. Sistema di navigazione e mappatura (SLAM & Nav2)
*   **SLAM Toolbox (Online Async):** Genera la mappa globale (`/map`) in tempo reale elaborando i dati del Lidar 2D e calcolando le trasformazioni (TF) necessarie per localizzare il robot.
*   **Navigation2 (Nav2):** Gestisce la navigazione point-to-point. I parametri (`nav2_params.yaml`) sono stati ottimizzati per consentire al TurtleBot di muoversi in spazi stretti e mitigare i problemi di deriva odometrica.
 
### 2. Algoritmo di esplorazione autonoma (Explore Lite)
L'esplorazione autonoma è demandata a **Explore Lite**, un algoritmo basato sull'esplorazione delle frontiere (Frontier Exploration). 
Il nodo analizza la *global_costmap* e identifica i bordi tra le celle note (spazio libero) e quelle ignote. È stato ottimizzato per dare forte priorità alle frontiere vicine, riducendo l'esitazione e i comportamenti a pendolo durante la scoperta del labirinto.
 
### 3. Logica di raccolta e percezione
Il nodo custom `maze_solver` funge da supervisore dell'intero obbiettivo. Utilizza `cv2.aruco` per elaborare i feed della telecamera RGB e stimare le pose 3D (Tramite `solvePnP`) dei marker rispetto al frame della telecamera (`camera_rgb_optical_frame`). Sfruttando `tf2_ros`, proietta poi le coordinate locali sulla mappa globale.
 
#### La Macchina a Stati Finita (FSM)
Il nodo opera seguendo una logica a stati per garantire un comportamento fluido:
*   **Stato 1: EXPLORING:** Il nodo delega il controllo a *Explore Lite*, limitandosi ad analizzare il feed video alla ricerca di marker.
*   **Stato 2: APPROACHING_KEY:** Quando viene individuata la chiave desiderata (o viene richiamata dalla memoria), *Explore Lite* viene messo in pausa (`/explore/resume: False`). Il nodo invia un goal asincrono direttamente a Nav2 tramite Action Client.
 
#### Memoria spaziale e raccolta per prossimità
*   **Raccolta Logica:** Il robot non collide con il marker. Il nodo calcola costantemente la profondità in asse Z del marker. Raggiunta una soglia di sicurezza (< 65 cm), il goal di Nav2 viene annullato attivamente per prevenire crash contro gli ostacoli fisici della costmap locale, validando il raggiungimento della chiave.
*   **Memoria Spaziale:** Durante lo stato di *EXPLORING*, se il robot avvista una chiave che non deve ancora raccogliere (es. vede la chiave 3 mentre cerca la 1), ne calcola le coordinate globali e le immagazzina in memoria. Quando sarà il momento di cercare quella specifica chiave, la FSM controllerà la memoria: in caso di riscontro positivo, eviterà una riesplorazione casuale e invierà un goal diretto a quelle coordinate, dimostrando di saper riconoscere l'ambiente circostante.
 
---
 
## Requisiti di sistema
Il progetto è stato sviluppato in un ambiente containerizzato per garantire la massima riproducibilità. L'unico requisito di sistema è avere installato **Docker** sul proprio computer.
Non sono richieste specifiche configurazioni hardware aggiuntive, né versioni locali di ROS 2 installate sull'host.
 
---
 
## Guida rapida all'avvio
 
### 1. Build e configurazione iniziale
Il progetto sfrutta Docker per installare automaticamente tutte le dipendenze e pre-compilare il workspace (`colcon build`) durante la creazione dell'immagine, risultando immediatamente pronto all'uso al primo avvio.
 
Posizionarsi nella directory radice del progetto (`mr_26_04def`) e avviare all'interno del docker workspace (`docker_ws`) il processo di costruzione dell'immagine:
"
./build.sh

Riposizionarsi all'interno della directory radice del progetto (`mr_26_04def`) per effettuare la clonazione delle repository tramite lo script dedicato:
./clone_repos.sh

Accedere al container tramite lo script dedicato:
./run.sh

Una volta all'interno del container, avviare l'infrastruttura di base (Gazebo, Rviz, SLAM, Nav2 ed Explore Lite) utilizzando il launch file master:
colcon build
source install/setup.bash
ros2 launch turtlebot_logic main.launch.py

I vari nodi si avvieranno sequenzialmente per non sovraccaricare il sistema. Attendere circa 15 secondi affinchè Rviz mostri il robot, le costmap e la mappa in tempo reale.

### 2. Avvia logica autonoma (Terminale 2)
Mentre il terminale 1 è in esecuzione, aprire un secondo terminale e posizionarsi nuovamente nella directory radice del progetto (mr_26_04def).
Accedere all'ambiente dello stesso container già in esecuzione:

./exec.sh

Avviare la FSM per dare avvio alla ricerca e raccolta delle chiavi tramite il comando:

ros2 run turtlebot_logic maze_solver

Da questo momento nel terminale 2 sarà possibile monitorare in tempo reale il pensiero del robot:
-Avvistamento delle frontiere
-Memorizzazione delle chiavi trovate casualmente durante l'esplorazione
-I cambi di stato (es. Navigazione autonoma: ripresa, Navigazione autonoma: stopped ecc.)
-Avvenuta raccolta per le nuove chiavi
