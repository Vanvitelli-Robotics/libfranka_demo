# Setup LIBFRANKA Vanvitelli
**Aggiornato a: novembre 2021**

La guida per il setup del Panda robot fornita dalla Franka Emika è piuttosto esaustiva pertanto, dove possibile, sono riportati i riferimenti alle pagine web per gli step da eseguire. 

1. Clonare la repository Panda_lab del laboratorio :     
>.git clone "url_repo_lab"

Nota: se si desidera utilizzare l'ultima versione di libfranka serve clonare la repo di github indicata nella guida: https://frankaemika.github.io/docs/installation_linux.html#building-libfranka.

2. Compilare il codice:
    1. Installare le dipendenze: sudo apt install build-essential cmake libpoco-dev libeigen3-dev
    2. Portarsi nella cartella Panda_lab: cd path/to/Panda_lab
    3. Creare il makefile: cmake -S . -B ./build 
    4. Build: cd ./build && cmake --build . 

3. Setup del real-time kernel: https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel

In data 10 Novembre la patch real-time è già installata sul computer "robo-lab" del laboratorio. 


# Modifiche alla repo libfranka ufficiale

Cambiamenti rispetto alla repo ufficiale di libfranka:

 1. E' stata creata la directory CustomPrograms. La directory CustomPrograms è pensata per contenere i file C++ realizzati dallo studente e per separarli dagli esempi di libfranka contenuti nella directory examples.

 2. E' stato aggiunto e configurato il CMakeList.txt della directory CustomPrograms. Il CMakeLists.txt della directory CustomPrograms è necessario per compilare i programmi C++ realizzati dallo studente. Ogni programma C++ che si vuole compilare va dichiarato nel CMakeLists.txt. E' consigliato rifarsi alla documentazione ufficiale di cmake per chiarimenti sul file. 

 3. E' stato modificato il root CMakeList.txt (CMakeList.txt contenuto in Panda_lab). Il root CMakeLists.txt è stato modificato in modo da includere CustomPrograms tra le subdirectory da buildare.

 4. E' stata creata la subdirectory CustomLibrary. La directory CustomLibraries è pensata per contenere eventuali librerie C++ scritte dallo studente. All'interno della directory è stato inserito un esempio utile a capire come configurare il CMakeLists.txt (StateSaver).

 5. E' stato aggiunto e configurato il file CmakeLists.txt della directory CustomLibrary. Il CMakeLists.txt della directory CustomPrograms è necessario per compilare le librerie C++. Ogni volta che si vuole creare una libreria questa va dichiarata nel CMakelists.txt
 
 6. E' stato creato il programma C++ EigenExamples nella directory CustomPrograms. Il programma EigenExamples mostra l'utilizzo di base della libreria Eigen3. 

# Scrivere, Compilare e Lanciare un programma: 

Innanzitutto, si consiglia di utilizzare VScode per scrivere i propri programmi in quanto le estensioni (ex CMake,C++ IntelliSense) dedicate al linguaggio C++ rendono più semplice il lavoro.

Gli step per scrivere, compilare e lanciare un algoritmo di controllo del Panda robot sono molto semplici:

1. Scrivere il programma C++ nella directory CustomPrograms. Nel caso in cui si ritenga necessario aggiungere una propria libreria scrivere questa nella directory CustomLibrary.

2. Aggiungere il file da compilare al set CUSTOMPROGRAMS nel CMakeLists.txt della directory CustomPrograms e specificare le librerie da linkare. 

    - Se è stata scritta una libreria nella directory CustomLibrary è necessario modificare il CMakeLists.txt della directory CustomLibrary in modo che tale libreria sia compilata. Solo una volta fatto questo passaggio la libreria può essere linkata al programma C++ che implementa l'algoritmo di controllo. 

3. Una volta modificati i file CMakeLists.txt è sufficiente eseguire:
    >.cd /path/to/Panda_lab/build
    >.make

4. Per lanciare il file eseguibile:
    >.cd /path/to/Panda_lab/build/CustomPrograms     
    >./nomefile ip_robot.



 