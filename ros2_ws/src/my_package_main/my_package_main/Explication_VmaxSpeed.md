# Vitesse maximale du centre effectif

Thomas Eyer
25/11/2026

## Introduction

Ce document explique d'où vient la formule utilisée dans la propriété `Kinematics.max_effective_speed` (`kinematics.py`) :

```
v_max_eff = v_max / sqrt(1 + (L / (2·D))²)
```

avec :

| Symbole | Signification | Unité |
|---|---|---|
| `v_max` | Vitesse maximale d'une roue (valeur absolue) | m/s |
| `L` | Distance entre les deux roues | m |
| `D` | Décalage du centre effectif devant le centre du robot | m |

Cette valeur est la vitesse maximale à donner à ORCA, car c'est la plus grande vitesse du centre effectif que les moteurs peuvent réaliser **quel que soit le cap** du robot.

Référence : Snape et al., « Smooth and Collision-Free Navigation for Multiple Robots Under Differential-Drive Constraints », IROS 2010, section III-B.

## Rappel : le centre effectif

Le centre `q` d'un robot différentiel n'est pas pilotable librement : il ne peut pas se déplacer latéralement. On pilote donc un point virtuel, le **centre effectif** `p`, situé à une distance `D` devant `q` dans la direction du cap `θ` :

```
p = q + D·(cos θ, sin θ)
```

Ce point est **holonome** : on peut lui imposer n'importe quelle vitesse `v = (Ẋ, Ẏ)`. ORCA raisonne sur ce point, avec un disque de rayon `R = r + D`.

La difficulté est que les moteurs sont limités. Il faut donc savoir quelles vitesses `v` sont réellement atteignables.

## Étape 1 : décomposer la vitesse dans un repère lié au robot

On décompose `v` selon deux directions orthogonales qui tournent avec le robot :

- `heading = (cos θ, sin θ)` : le cap (direction avant).
- `normal = (−sin θ, cos θ)` : orthogonale au cap, vers la gauche.

```
v_f = v · heading        (composante avant/arrière)
v_n = v · normal         (composante latérale)
```

Ces deux composantes ont une interprétation physique :

- `v_f = (v_g + v_d) / 2` : la vitesse linéaire du robot.
- `v_n = D·ω` : le centre effectif est à une distance `D` du centre, donc une rotation à la vitesse angulaire `ω` le fait glisser latéralement de `D·ω`.

D'où la vitesse angulaire à commander :

```
ω = v_n / D
```

## Étape 2 : exprimer les vitesses des roues

Le modèle différentiel donne `ω = (v_d − v_g) / L` et `v_f = (v_g + v_d) / 2`. En inversant :

```
v_g = v_f − (L/2)·ω
v_d = v_f + (L/2)·ω
```

En remplaçant `ω = v_n / D` et en posant `k = L / (2·D)` :

```
v_g = v_f − k·v_n
v_d = v_f + k·v_n
```

C'est ce que calcule `Kinematics.wheel_speeds`, sans avoir besoin d'inverser la matrice `M(θ)` de l'article.

## Étape 3 : écrire la contrainte moteur

Chaque roue est limitée : `|v_g| ≤ v_max` et `|v_d| ≤ v_max`.

On utilise l'identité `max(|a − b|, |a + b|) = |a| + |b|`. Les deux contraintes se résument alors à une seule :

```
|v_f| + k·|v_n| ≤ v_max
```

Dans le plan `(v_f, v_n)`, cette inégalité décrit un **losange** (un carré tourné si `k = 1`) de sommets :

```
(+v_max, 0)   (−v_max, 0)   (0, +v_max/k)   (0, −v_max/k)
```

Ce losange est l'ensemble des vitesses atteignables. Quand le robot tourne, le repère `(heading, normal)` tourne avec lui : le losange tourne donc avec `θ` dans le repère monde, mais **sa forme et ses dimensions ne changent pas**.

## Étape 4 : le cercle inscrit

ORCA ne sait pas gérer une forme qui dépend de `θ`. Il lui faut une limite simple sur la norme de la vitesse. On choisit le plus grand cercle centré en `0` qui tient entièrement dans le losange. Comme il est centré en `0`, il est invariant par rotation : il est donc valable pour tous les caps.

Son rayon est la distance de l'origine au bord le plus proche du losange. Prenons le bord `v_f + k·v_n = v_max`. La distance d'un point à une droite `a·x + b·y = c` est `c / sqrt(a² + b²)`, ici avec `a = 1`, `b = k`, `c = v_max` :

```
rayon = v_max / sqrt(1 + k²)
      = v_max / sqrt(1 + (L / (2·D))²)
```

Les quatre bords du losange sont symétriques : ils sont tous à cette même distance de l'origine. Toute vitesse de norme inférieure ou égale à ce rayon est donc réalisable, quel que soit `θ`.

## Cas particulier du papier : L = 2·D

Si `D = r` et que le robot est un disque dont les roues sont sur le bord (`L = 2·r`), alors `k = 1`. Le losange est un carré et :

```
v_max_eff = v_max / sqrt(2) ≈ 0,707 · v_max
```

Exemple avec `v_max = 0,5 m/s` : `v_max_eff ≈ 0,354 m/s`.

## Influence de D

| Choix de `D` | `k = L/(2D)` | `v_max_eff` | Effet |
|---|---|---|---|
| `D` petit | grand | petit | Losange très étroit : peu de vitesse latérale possible, rotations très rapides requises |
| `D = L/2` | 1 | `v_max / sqrt(2)` | Cas du papier |
| `D` grand | petit | proche de `v_max` | Plus de vitesse disponible, mais rayon effectif `R = r + D` plus grand, donc plus encombrant pour ORCA |

Le choix de `D` est un compromis entre vitesse disponible et encombrement dans les passages étroits.

## Utilisation dans le code

```python
kinematics = Kinematics(wheel_base, effective_offset, robot_radius, max_wheel_speed)

kinematics.effective_radius      # R = r + D, rayon à donner à ORCA
kinematics.max_effective_speed   # vitesse max à donner à ORCA
```

Si ORCA utilisait directement `v_max` (vitesse brute d'une roue) comme limite, il pourrait renvoyer une vitesse non réalisable dans certaines directions. `Kinematics.wheel_speeds` la réduirait alors proportionnellement (saturation), ce qui fonctionne mais dégrade la fluidité et les garanties d'évitement d'ORCA. D'où l'importance de lui donner la bonne limite en amont.

Le test `test_max_effective_speed_is_always_feasible` (`test/test_kinematics.py`) vérifie numériquement cette propriété sur tous les caps et toutes les directions.
