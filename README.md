# Setup LIBFRANKA Vanvitelli
**Aggiornato a: novembre 2021**

La guida al setup del Panda robot fornita dalla Franka Emika è piuttosto esaustiva pertanto, dove possibile, per gli step da eseguire sono riportati i riferimenti alle loro pagine web. 
 
In particolare, gli step sono: 

1. Clonare la repository PandaPrograms del laboratorio :     
   >git clone "url_repo_lab"

Nota: se si desidera utilizzare l'ultima versione di libfranka serve clonare la repo di github indicata nella [guida setup libfranka](https://frankaemika.github.io/docs/installation_linux.html#building-libfranka). 
    
2. Compilare il codice:
    1. Installare le dipendenze: 
        >sudo apt install build-essential cmake libpoco-dev libeigen3-dev
    
    2. Nella directory PandaPrograms creare la cartella build:
        >mkdir build    
        >cd build 
        
    3. Creare i makefile:
        >cmake ..

    4. Compilare: 
        >make
    
3. Setup del real-time kernel: [guida patch real-time kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). In data 10 Novembre la patch real-time è già installata sul computer "asusrobot" del laboratorio.

4. Configurare la connessione con il robot: [guida setup network connection](https://frankaemika.github.io/docs/getting_started.html#setting-up-the-network)

5. Leggere il pdf "Manuale_Panda" scritto dal collega Vetrella. 


# Modifiche alla repo libfranka ufficiale

Cosa cambia tra la repo appena clonata e quella ufficiale di libfranka? Con riferimento alla repository ufficiale di Novembre 2021: 

 1. E' stata creata la directory CustomPrograms. Tale directory è pensata per contenere i file C++ realizzati dallo studente e per separarli dagli esempi di libfranka contenuti nella directory examples.

 2. E' stato aggiunto e configurato il CMakeList.txt della directory CustomPrograms. Il file è necessario per compilare i programmi C++ realizzati dallo studente. Ogni programma C++ che si vuole compilare va dichiarato nel CMakeLists.txt. E' consigliato rifarsi alla documentazione ufficiale di cmake per chiarimenti sul file. 

 3. E' stato modificato il root CMakeList.txt (CMakeList.txt contenuto in PandaPrograms). Il root CMakeLists.txt è stato modificato in modo da includere CustomPrograms tra le subdirectory da buildare.

 4. E' stata creata la subdirectory CustomLibrary. La directory CustomLibraries è pensata per contenere eventuali librerie C++ scritte dallo studente. All'interno della directory è stato inserito un esempio utile a capire come configurare il CMakeLists.txt (libreria StateSaver).

 5. E' stato aggiunto e configurato il file CmakeLists.txt della directory CustomLibrary. Il file è necessario per compilare le librerie C++. Ogni volta che si vuole creare una libreria questa va dichiarata nel CMakelists.txt
 


# Compilare e Lanciare un programma: 

Innanzitutto, si consiglia di utilizzare VScode per scrivere i propri programmi in quanto le estensioni (ex CMake,C++ IntelliSense) dedicate al linguaggio C++ rendono più semplice il lavoro.

Per compilare e lanciare un programma serve:

1. Scrivere il programma C++ nella directory CustomPrograms. Prima di scrivere il proprio algoritmo C++ sfruttando la libreria libfranka è suggerito dare uno sguardo agli esempi inseriti nella directory CustomPrograms e agli esempi contenuti nella directory examples. In particolare, il programma Cartesian_Force_Impedance_Control.cpp è frutto della tesi del collega Vetrella, motivo per cui nella directory root è stato incluso anche il pdf della sua tesi.    Nel caso in cui si ritenga necessario aggiungere una propria libreria scrivere questa nella subdirectory CustomLibrary.

2. Aggiungere il file da compilare al set CUSTOMPROGRAMS nel CMakeLists.txt della directory CustomPrograms e specificare le librerie da linkare. 

    - Se è stata scritta una libreria nella directory CustomLibrary è necessario modificare il CMakeLists.txt della directory CustomLibrary in modo che tale libreria sia compilata. Solo una volta fatto questo passaggio la libreria può essere linkata al programma C++ che implementa l'algoritmo di controllo. 

3. Una volta modificati i file CMakeLists.txt è sufficiente eseguire:
    >cd /path/to/PandaPrograms/build/CustomPrograms
    >make

4. Per lanciare il file eseguibile:
    >cd /path/to/PandaPrograms/build/CustomPrograms     
    >./nomefile ip_robot.



 