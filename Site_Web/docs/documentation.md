# Documentation

Cette partie sert à éclairer le code du projet sur l'album photo. 

## L'objectif

L'objectif de ce projet est d'avoir une visionneuse de photographie automatique sur notre carte STM32 lisant des photos à partir d'une carte SD, le tout fonctionnant avec FreeRTOS.

## Le fonctionnement

Ce programme permet d'afficher des images se trouvant au format .BMP (*Bitmap*) qui sont dans un dossier **Media** dans le répertoire racine d'une carte SD. Des images se trouvent sur le dépôt GitHub ici : [Media](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/tree/main/Media_test).
> Le code proposé est largement inspiré de celui du projet *LTDC_PicturesFromSDCard* proposé par STM.

> **ATTENTION** Les images ne doivent pas dépaser une taille de 480x272. Cependant, le programme accepte les images plus petites.

---
## Le diagramme d'interaction
Vous trouverez ci-dessous le diagramme d'interaction entre les différentes tâches du projet.

![Interaction_taches](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/diag_inter_album_photo.png?raw=true)

> Il se trouve que la tâche démarrage n'apparaît pas car je n'ai pas réussi à la faire fonctionner. Elle n'entre alors plus en jeu dans le fonctionnement du programme.

---
## L'utilisation de la carte SD et de FreeRTOS

### Affichage d'une image

L'affichage des images se fait en utilisant les 2 calques (*layers*) de l'écran en alternance. 
![affichage_code](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/affich_image.png?raw=true)
Le code sélectionne l'image via le compteur qui est défini dans la liste des images disponibles sur la carte SD. Ensuite, il vient stocker l'image dans un buffer avant de l'afficher sur un des calques (*BSP_LCD_DrawBitmap(...)*).

### Transitions

Les transitions sont réalisées lors du changement d'image et il s'agit de fondus au noir. L'image actuelle s'assombrit jusqu'à ce que l'écran affiche du noir, puis l'image suivante s'éclaircit jusqu'à appararaître. Ce type de fondu est choisi car il permet de ne stocker dans le *buffer* mémoire de la carte STM32 qu'une image à la fois. Les transitions s'effectue en jouant aussi sur les deux calques (ici le calque 0).

![transition1](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/transition1.png?raw=true)

![transition2](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/transition2.png?raw=true)

Pour forcer la transition d'une image, on utilise une messagerie qui vient modifier la valeur sur compteur en fonction de là où l'utilisateur a appuyé sur l'écran.

![IHM](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/blob/main/Images/modif_counter.png?raw=true)

### Difficultés

Il a été difficile de mettre en place l'utilisation de FreeRTOS en même temps que l'utilisation de la carte SD. Cependant, une fois ce blocage dépassé (grâce à l'intervention (divine ?) de M. Juton), il ne restait plus qu'à faire la transposition des bibliothèques proposées par STM pour la lecture de fichiers bitmap. Cependant, il fallait faire bien attention à n'importer que les bibliothèques nécessaires et à commenter les lignes inutiles et qui faisaient doublon avec d'autres bibliothèques déjà utilisées.