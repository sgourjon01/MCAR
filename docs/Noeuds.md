# Nœuds

Mathias Bouchut--Loubet
10/11/2026

## Introduction

Ce document décrit les nœuds utilisés dans le projet, ainsi que leur organisation et leurs interactions.

## Rappel sur les nœuds ROS 2

Les nœuds sont des blocs de logique élémentaires dans l'architecture ROS 2.

Chaque nœud contient un programme chargé de contrôler certains éléments logiciels et/ou matériels du système.

L'organisation de plusieurs nœuds et leur communication permettent de construire des systèmes complexes.

Avec ROS 2, l'objectif est de décomposer le système en plusieurs nœuds indépendants, chacun ayant une responsabilité précise. Ces nœuds communiquent ensuite entre eux afin de former le système complet souhaité.

## Les nœuds utilisés

Le projet utilise trois types de nœuds :

- `Manager`
- `Camera`
- `Robot`

On veillera à respecter la convention de nommage `PascalCase` pour l'ensemble des nœuds.

Le rôle de chaque nœud ainsi que son organisation sont détaillés ci-dessous.

Pour plus d'informations sur les messages échangés entre les nœuds, consulter le document [Topics.md](Topics.md).

### Manager

Le nœud `Manager` gère l'ensemble de la flotte de robots.

C'est lui qui attribue les tâches aux différents robots, notamment en leur indiquant quel colis déplacer et vers quelle destination.

Ce nœud fournit également une IHM permettant de visualiser facilement l'état global du système, d'attribuer des tâches et de faciliter le débogage.

Le nœud `Manager` :

- **Souscrit** aux topics `robot_pose` et `robot_state` afin de récupérer les informations globales du système.
- **Publie** sur le topic `robot_task` afin de transmettre des ordres aux robots.

### Camera

Le nœud `Camera` gère la caméra ainsi que la reconstruction des positions et des vitesses des robots.

Il récupère les informations de position et de vitesse (pose) des robots à partir des images de la caméra, puis les publie en continu.

Le nœud `Camera` :

- **Publie** les poses des robots sur le topic `robot_pose`.

### Robot

Le nœud `Robot` gère un robot particulier, identifié par un identifiant entier compris entre 1 et 4 inclus.

Un seul programme est développé pour ce type de nœud. Cependant, lors de son lancement, le nœud reçoit en paramètre l'identifiant du robot auquel il est associé.

Il faut donc lancer autant d'instances du nœud `Robot` qu'il y a de robots physiques dans le système.

Le nœud `Robot` gère la réception des ordres envoyés par le `Manager` et leur bonne exécution. Pour cela, il pilote le robot et assure notamment l'évitement des obstacles.

Le nœud `Robot` :

- **Souscrit** aux topics `robot_task` et `robot_pose` afin de recevoir les ordres du manager et de connaître sa position.
- **Publie** son état sur le topic `robot_state`.