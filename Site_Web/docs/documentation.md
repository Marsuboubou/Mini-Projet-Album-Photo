# Documentation

Cette partie sert à documenter le code du projet sur l'album photo. 

## Le fonctionnement

Ce programme permet d'afficher des images se trouvant au format .BMP (*Bitmap*) qui sont dans un dossier **Media** dans le répertoire racine d'une carte SD. Des images se trouvent sur le dépôt GitHub ici : [Media](https://github.com/Marsuboubou/Mini-Projet-Album-Photo/tree/main/Media_test).
> Le code proposé est largement inspiré de celui du projet *LTDC_PicturesFromSDCard* proposé par STM.

> **ATTENTION** Les images ne doivent pas dépaser une taille de 480x272. Cependant, le programme accepte les images plus petites.

---
## Le diagramme d'interaction
Vous trouverez ci-dessous le diagramme d'interaction entre les différentes tâches du projet.



> Il se trouve que la tâche démarrage n'apparaît pas car je n'ai pas réussi à la faire fonctionner. Elle n'entre alors plus en jeu dans le fonctionnement du programme.

---
## L'utilisation de la carte SD et de FreeRTOS

### Affichage d'une image

L'affichage des images se fait en utilisant les 2 calques (*layers*) de l'écran en alternance.

### Transitions

Les transitions sont réalisées lors du changement d'image et il s'agit de fondus au noir. L'image actuelle s'assombrit jusqu'à ce que l'écran affiche du noir, puis l'image suivante s'éclaircit jusqu'à appararaître. Ce type de fondu est choisi car il permet de ne stocker dans le *buffer* mémoire de la carte STM32 qu'une image à la fois.

### Difficultés

Il a été difficile de mettre en place l'utilisation de FreeRTOS en même temps que l'utilisation de la carte SD. Cependant, une fois ce blocage dépassé, il ne restait plus qu'à faire la transposition des bibliothèques proposées par STM pour la lecture de fichiers .BMP.