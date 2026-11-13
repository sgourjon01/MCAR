# Topics

Mathias Bouchut--Loubet
09/11/2026

## Introduction

Ce document décrit les topics utilisés dans le projet, ainsi que les messages qui y transitent.

## Rappel sur les topics ROS 2

Les topics sont des canaux de communication entre les nœuds ROS 2.

Un nœud peut publier et/ou s'abonner à un ou plusieurs topics simultanément.

Le rôle d'un topic est de transmettre des informations entre les différents nœuds du système.

Les messages qui transitent sur les topics doivent respecter un format particulier, connu à l'avance par les nœuds qui les publient ou les reçoivent. Il est donc essentiel de définir une convention commune pour assurer la bonne communication entre les différents nœuds. C'est l'objectif de ce document.

## Les topics utilisés

Le projet utilise trois topics :

- `robot_task`
- `robot_state`
- `robot_pose`

On veillera à respecter la convention de nommage `snake_case` pour l'ensemble des topics.

Le rôle de chaque topic est détaillé ci-dessous.

### robot_task

**Type des messages : `float[n]`**  
**Type ROS 2 :** `std_msgs.msg.Float32MultiArray`  
taille n variable

Ce topic permet de transmettre des ordres du manager vers les robots.

Typiquement, les ordres permettant à un robot d'aller récupérer un colis à un emplacement donné transitent par ce topic.

Les données sont envoyées sous la forme d'un tableau de nombres flottants ordonnés, composé de deux parties :

- Une en-tête (header).
- Un corps de message (payload).

#### En-tête du message

L'en-tête contient toujours deux éléments :

1. L'identifiant du robot (`int` converti en `float`).
2. Le type de tâche (`int` converti en `float`), défini par l'énumération `TaskType`.

L'identifiant du robot permet de déterminer à quel robot l'ordre est destiné.

Le type de tâche permet d'identifier la commande à exécuter. L'énumération `TaskType` définit les différents types de tâches disponibles.

Ce type détermine également le nombre d'éléments présents dans le corps du message, ainsi que la manière dont ces éléments doivent être interprétés.

#### Corps du message

Le corps du message contient plusieurs éléments dont la nature et le nombre dépendent du type de tâche transmis.

Pour interpréter correctement ces données, le récepteur doit adapter leur lecture en fonction du type de tâche indiqué dans l'en-tête.

#### Les types de tâches

Le tableau ci-dessous décrit les différents types de tâches ainsi que les éléments attendus dans leur corps de message.

| Description | Type de tâche | Corps du message |
|---|---|---|
| Ordre de déplacement d'un paquet | `TaskType.MOVE_PACKET` | Position X du paquet [mm] ; Position Y du paquet [mm] ; Position X de la zone de dépôt [mm] ; Position Y de la zone de dépôt [mm] |

### robot_state

**Type des messages : `int[2]`**  
**Type ROS 2 :** `std_msgs.msg.Int8MultiArray`

Ce topic permet de transmettre l'état d'un robot vers le manager.

Les robots publient régulièrement leur état afin que le manager puisse disposer d'une vue d'ensemble du système et assurer le suivi des opérations.

Chaque message est composé de deux entiers :

1. L'identifiant du robot.
2. L'état actuel du robot.

L'énumération `RobotState` définit les différents états possibles d'un robot.

Les états disponibles sont détaillés ci-dessous :

| État | Description |
|---|---|
| `RobotState.WAIT` | Le robot est en attente. Aucune tâche ne lui est actuellement assignée. |
| `RobotState.SEARCHING_PACKET` | Le robot se dirige vers le colis afin de le récupérer. |
| `RobotState.TRANSPORTING` | Le robot transporte le colis vers sa destination. |
| `RobotState.TASK_DONE` | La tâche est terminée. Le robot attend la confirmation du manager avant d'accepter un nouvel ordre ou de passer à l'état `RobotState.WAIT`. |

### robot_pose

**Type des messages : `float[7]`**  
**Type ROS 2 :** `std_msgs.msg.Float32MultiArray`

Ce topic permet de transmettre les positions et les vitesses des robots, mesurées par la caméra.

La caméra récupère les positions réelles des robots. Ces informations sont ensuite mises à disposition du manager et des robots afin qu'ils puissent connaître leur position dans l'environnement.

Chaque message est composé de sept nombres flottants :

1. L'identifiant du robot.
2. La position X du robot [mm].
3. La position Y du robot [mm].
4. L'orientation du robot [rad].
5. La vitesse linéaire selon X [mm/s].
6. La vitesse linéaire selon Y [mm/s].
7. La vitesse angulaire [rad/s].

| Index | Donnée | Unité |
|---:|---|---|
| 0 | Identifiant du robot | - |
| 1 | Position X | mm |
| 2 | Position Y | mm |
| 3 | Orientation | rad |
| 4 | Vitesse linéaire X | mm/s |
| 5 | Vitesse linéaire Y | mm/s |
| 6 | Vitesse angulaire | rad/s |