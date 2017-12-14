Il pacchetto *politocean* va inserito all'interno di: *<ros_workspace>/src/* <br />

Eseguire il comando <br />
`catkin_make` <br />
all'interno di *<ros_workspace>* per aggiornare l'environment con le modifiche apportate <br />

Per lanciare i nodi creati nel pacchetto *politocean* usare il comando <br />
`rosrun politocean <nome_nodo>` <br />
N.B. gli script nella sottocartella *scripts* vanno marcati come eseguibili per poter funzionare <br />

Per avviare il nodo che abilita RosSerial eseguire il comando <br />
`rosrun rosserial_python serial_node.py /dev/<nome_porta_seriale>` <br />

