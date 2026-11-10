# MCAR
Projet ROS robots AMR

## Bonne pratiques
- Attention à "colcon build" toujours dans le répertoire de travail (ros2_ws)
- les fichiers pythons correspondant à des noeuds se nomme comme suit: "my_node_xxx.py"
- les autres fichier python: selon le nom de la classe (en minuscule)
- un fichier par classe, des noms de classe selon une convention (pascal case/cmale case??)
- les conventions python: ma_variable_dummy, ma_fonction_dummy...

## Règle git:
- push seulement le dossier src
- chacun sa branche, puis merge (pull request) avec **dev** une fois les codes viables
- Seulement les version fonvtionel de dev serons merge avec le main (avec un pull request)
- documenter, faire des markdown pour expliquer sa logique, ses conventions (dossier docs)

## A modifier à chaque création de noeuds:
Setup.py: rajouter les noeuds qu'on veut utiliser:  
```
entry_points={
        'console_scripts': [
            'motor = smarties.motor:main',
            'teleop_keyboard = smarties.teleop_keyboard:main',
            'leddroite = smarties.LedDroite:main',
            'ledgauche = smarties.LedGauche:main',
            'sonar = smarties.sonar:main',
            'bouton = smarties.bouton:main',   
            'capteur_ligne = smarties.ligne:main' 
        ],
    },
```
    
## A modifier à chaque nouvelle librairie utilisé
package.xml: rajouter les lib qui sont (et seront) utilisé

## Après chaque modifications:
colcon build (à la racine ros2_ws)

## Commandes utiles:
Pour lancer un neoud: ros2 run my_package_main my_node_xxx






