
# Conventions

Mathias Bouchut--Loubet
24/11/2026

## Introduction

Ce document définit les conventions utilisées dans l'ensemble du projet.

Ces conventions sont essentielles pour garantir la cohérence et le bon fonctionnement des échanges entre les différents nœuds du système.

## Unités

Toutes les grandeurs physiques doivent être stockées et transmises entre les nœuds en utilisant les unités suivantes :

- **Longueur :** mm (millimètres)
- **Temps :** s (secondes)

Par conséquent, les vitesses sont exprimées en **mm/s**.

## Repère

Toutes les positions utilisées ou représentées dans le repère fixe du sol doivent respecter un système de coordonnées commun à l'ensemble du projet.

Ce repère est défini de la manière suivante :

- **Origine :** en haut à gauche de l'écran.
- **Axe X :** orienté vers la droite.
- **Axe Y :** orienté vers le bas.
- **Axe Z (si nécessaire) :** orienté vers l'intérieur de l'écran.

Toutes les coordonnées doivent être positives ou nulles. Ainsi, tout objet sur la carte ne peut pas être positionné avec une coordonnée négative.

### Cas de la réalité

Dans le monde réel, il n'y a pas d'écran permettant de définir directement l'origine du repère. Nous utiliserons donc un marqueur ArUco fixe au sol pour définir ce repère.

Ce marqueur ArUco permet de fixer :
- **L'origine :** définie par la position du marqueur.
- **Les axes X et Y :** définis par l'orientation du marqueur.
- **L'axe Z (si nécessaire) :** orienté verticalement, vers le bas.

Le marqueur ArUco doit être positionné et orienté de manière à garantir que les coordonnées des robots présents sur la carte soient toujours positives ou nulles.

Ainsi, le repère utilisé dans la réalité reste cohérent avec celui utilisé pour les représentations à l'écran.
