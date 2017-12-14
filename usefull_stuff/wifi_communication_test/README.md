I codici in python sono nodoi di ROS, vanno usati come tali (vedi README della cartella di ROS) <br />

Nella cartella srv c'è la definizione del tipo di comunicazione tra service e client (chiamto *wifi*) <br />

*wifi_client.py* è un client (come dice il nome), lanciato senza argomenti richiede al service un dato *wifi* <br />
*wifi_service.py* è un service, aspetta una richiesta e risponde con il dato letto tramite wifi <br />
*wifi_publicher.py* è un nodo che continua a pubblicare un UInt8 letto da wifi sul topic *wifi_data*<br />

Il publicher legge un intero all'indirizzo 192.168.4.1/test <br />
Il service legge una tabella di valori (x,y) all'indirizzo 192.168.4.1/test_new <br />

Il codice in Arduino è testato su una scheda WeMos D1 R2, (non assicuro il funzionamento su altre schede) <br />
Crea una rete wifi chiamata wifi (password: password)
All'indirizzo 192.168.4.1/test viene pubblicata una variabile random compresa tra 0 e 100 <br />
All'indirizzo 192.168.4.1/test_new viene pubblicata la tabella di valori (x,y) riportata nel manuale della competizione <br />

Tutti gli errori di connessione o di lettura dati vengono pubblicati sul topic *errors* <br />
