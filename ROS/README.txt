il pacchetto "politocean" va inserito all'interno di: <ros_workspace>/src/
la cartella "launches" va inserita all'interno di: <ros_workspace>/

eseguire il comando
>> catkin_make
all'interno di <ros_workspace> per aggiornare l'environment con le modifiche apportate

per lanciare i nodi creati nel pacchetto "politocean" usare il comando
>> rosrun politocean <nome_nodo>
N.B. gli script nella sottocartella "scripts" vanno marcati come eseguibili per poter funzionare

per avviare il nodo che abilita RosSerial eseguire il comando
>> rosrun rosserial_python serial_node.py /dev/<nome_porta_seriale>

per avviare la lettura del video eseguire il lauch file tramite il comando
>> roslaunch video_stream.launch
all'interno di <ros_workspace>/lauches/

per visualizzare il video su schermo eseguire il comando
>> rosrun image_view image_view image:=/webcam/image_raw
