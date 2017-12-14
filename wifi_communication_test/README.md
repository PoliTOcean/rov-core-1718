I codici in python sono nodoi di ROS, vanno usati come tali (vedi README della cartella di ROS)

nella cartella srv c'è la definizione del tipo di comunicazione tra service e client (chiamto 'wifi')

wifi_client.py è un client (come dice il nome), lanciato senza argomenti richiede al service un dato 'wifi'
wifi_service.py è un service, aspetta una richiesta e risponde con il dato letto tramite wifi
wifi_publicher.py è un nodo che continua a pubblicare un UInt8 letto da wifi sul topic 'wifi_data'

Il publicher legge un intero all'indirizzo 192.168.4.1/test
Il service legge una tabella di valori (x,y) all'indirizzo 192.168.4.1/test_new

Il codice in Arduino è testato su una scheda WeMos D1 R2,(non assicuro il funzionamento su altre schede)
Crea una rete wifi chiamata wifi (password: password)
All'indirizzo 192.168.4.1/test viene pubblicata una variabile random compresa tra 0 e 100
All'indirizzo 192.168.4.1/test_new viene pubblicata la tabella di valori (x,y) riportata nel manuale della competizione

Tutti gli errori di connessione o di lettura dati vengono pubblicati sul topic 'errors'
