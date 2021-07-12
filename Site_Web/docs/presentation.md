# Présentation sur le SDMMC

Cette partie permet d'afficher la présentation sur le SDMMC sur un site web static. 

---
## Diapositive 1
![diapo1](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive1.PNG?raw=true)

---
## Diapositive 2
![diapo2](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive2.PNG?raw=true)

---
## Diapositive 3
![diapo3](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive3.PNG?raw=true)

On rappelle que le protocole SDMMC permet de faire communiquer une carte SD ou une carte MMC avec un microcontrôleur. Il est grandement inspiré du protocole SPI, protocole très utilisé. Il est donc important d'avoir quelques bases sur le SPI avant de comprendre le fonctionnement du SDMMC.

---
## Diapositive 4
![diapo4](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive4.PNG?raw=true)

On présent ainsi le protocole SPI rapidement avec son fonctionnement sur 4 fils. Cependant, il existe des configurations sur 3 fils où l'on regroupe le MOSI et le MISO sur un seul et unique fil. Ainsi, le SPI est flexible mais présentent plus de connexions que l'I2C.

---
## Diapositive 5
![diapo5](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive5.PNG?raw=true)

Le protocole SDMMC est basé sur une configuration SPI à 3 fils (3 wired). C'est un protocole qui permet de meilleures performances en terme de rapidité que le SPI. De plus, il est assez flexible car on peut paramétrer le bus de données en différents modes. Enfin, il prend en compte le DMA pour gagner en rapidité lorsque l'on souhaite transferer des données.

---
## Diapositive 6
![diapo6](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive6.PNG?raw=true)

On explique dans cette diapositive rapidement le fonctionnement général du SDMMC et sa syntaxe avec un bit de **START**, un bit de **STOP** et un **checksum** à la fin pour vérifier l'intégrité de ce qui a été transmis.

---
## Diapositive 7
![diapo7](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/Diapositive7.PNG?raw=true)
> Si vous avez des questions, vous pouvez m'envoyer un mail (marceau.bouchez@ens-paris-saclay.fr).

---