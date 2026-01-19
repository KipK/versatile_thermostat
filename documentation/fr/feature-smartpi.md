# L'algorithme SmartPI

- [L'algorithme SmartPI](#lalgorithme-smartpi)
  - [Principe de fonctionnement](#principe-de-fonctionnement)
  - [Qu'est-ce que ça fait concrètement ?](#quest-ce-que-ça-fait-concrètement-)
  - [Configuration](#configuration)
  - [Paramètres disponibles](#paramètres-disponibles)
  - [Cas d'utilisation recommandés](#cas-dutilisation-recommandés)
  - [Différences avec TPI et Auto-TPI](#différences-avec-tpi-et-auto-tpi)

## Principe de fonctionnement

L'algorithme **SmartPI** est un régulateur adaptatif qui apprend automatiquement le comportement thermique de votre pièce. Contrairement à TPI qui utilise des coefficients fixes, SmartPI s'adapte en permanence aux caractéristiques de votre installation.

### Comment ça marche ?

1. **Apprentissage continu** : À chaque cycle de chauffage, SmartPI observe comment la température évolue en fonction de la puissance appliquée
2. **Modélisation thermique** : Il construit un modèle mathématique qui représente votre pièce (inertie, déperditions, puissance du radiateur) via une régression robuste Theil-Sen
3. **Adaptation des gains** : Les paramètres du régulateur sont automatiquement ajustés via une heuristique basée sur la constante de temps thermique

```
┌─────────────────────────────────────────────────────┐
│                    SmartPI                          │
│                                                     │
│   Température ──► Apprentissage ──► Modèle pièce   │
│   Puissance   ──► (Theil-Sen)  ──► (a, b)          │
│                         │                           │
│                         ▼                           │
│               Vérification fiabilité tau            │
│                         │                           │
│                         ▼                           │
│               Calcul des gains (Kp, Ki)             │
│               via heuristique adaptative            │
│                         │                           │
│                         ▼                           │
│               Commande de puissance (%)             │
└─────────────────────────────────────────────────────┘
```

## Qu'est-ce que ça fait concrètement ?

- **Au démarrage** : SmartPI utilise des valeurs par défaut conservatrices
- **Après quelques heures** : Il commence à comprendre votre installation
- **Après quelques jours** : Le modèle est affiné et la régulation devient optimale

### Avantages
 
 ✅ **Pas de réglage manuel** : Pas besoin de trouver les bons coefficients Kint/Kext  
 ✅ **S'adapte aux changements** : Si vous changez de radiateur ou isolez, il se ré-adapte  
 ✅ **Évite les oscillations** : Réduction automatique des gains près de la consigne (near-band scheduling)  
 ✅ **Robuste aux perturbations** : L'algo ignore les variations brutales pour ne pas fausser le modèle  
 ✅ **Gestion de l'inertie** : Prend en compte la constante de temps thermique de votre pièce  
 ✅ **Anti-dépassement** : Protection 2-DOF PI et décharge douce de l'intégrale (sign-flip leak)
 ✅ **Fiabilité du modèle** : Utilise des gains "safe" et une rampe progressive du feed-forward
 ✅ **Anti-windup** : Intégration conditionnelle pour éviter la saturation

## Configuration

Pour activer SmartPI :

1. Créez ou modifiez un VTherm de type **switch** ou **vanne**
2. Dans **Sous-jacents**, sélectionnez l'algorithme **SmartPI**
3. Configurez les paramètres dans le menu **SmartPI**

## Paramètres disponibles

| Paramètre | Description | Valeur par défaut | Recommandation |
|-----------|-------------|-------------------|----------------|
| **Bande morte (°C)** | Zone autour de la consigne où aucune action n'est effectuée. Évite les micro-régulations | 0.05°C | 0.05 - 0.1°C |
| **Agressivité** | Facteur de réponse du régulateur. Plus haut = plus réactif (gains plus forts) | 0.5 | 0.3 (très doux) à 1.0 (très réactif) |
| **Utiliser le filtre de consigne** | Active/Désactive le filtre de consigne asymétrique. | Coché (Vrai) | Coché (recommandé) |

### Réglage de l'agressivité

L'agressivité influence directement les gains du régulateur (Kp). Une valeur plus haute donne des gains plus forts, donc une réponse plus rapide mais avec plus de risque d'oscillation.

- **0.3** : Réponse très douce, pour les pièces à forte inertie ou sujettes aux dépassements
- **0.5** : Équilibre performance/stabilité. Point de départ recommandé
- **1.0** : Réponse très réactive, pour les radiateurs électriques rapides sans inertie

L'algorithme réduit automatiquement les gains Kp et Ki dans une bande proche de la consigne (±0.5°C par défaut) grâce au **near-band scheduling**.

## Fonctionnement détaillé de l'algorithme

Le fonctionnement de SmartPI peut se découper en 5 étapes cycliques :

### 1. Mesure et Observation
À chaque cycle, l'algorithme collecte :
- La température actuelle ($T_{int}$)
- La température extérieure ($T_{ext}$)
- La puissance qui a été envoyée au radiateur ($u$)

### 2. Modélisation Hybride (Huber + Theil-Sen)
Il met à jour son modèle interne de la pièce défini par deux paramètres :
- **a** (Efficacité) : Combien de degrés je gagne par minute si je chauffe à 100%
- **b** (Déperdition) : Combien de degrés je perds par minute par degré d'écart avec l'extérieur

L'algorithme utilise une **stratégie d'apprentissage hybride** :
1. **Phase initiale (Tau non fiable)** : Utilise une estimation robuste basée sur la médiane et la MAD (Median Absolute Deviation) combinée à une méthode de Huber. Cette approche est très résiliente au bruit et permet une convergence rapide même avec peu de données.
2. **Phase stable (Tau fiable)** : Bascule automatiquement vers une **régression robuste Theil-Sen** sélective dès que le modèle est stabilisé.

Détails de l'apprentissage :
- **b** est appris principalement pendant les phases OFF (u < 5%), quand le refroidissement est observable
- **a** est appris principalement pendant les phases ON (u > 20%), quand le chauffage est significatif
- **Rejet des outliers** : Utilise la MAD pour détecter et rejeter les échantillons avec des résidus anormaux (ex: apport solaire, glitch capteur)
- **Cohérence de l'intercept** : Vérifie que la régression linéaire passe près de l'origine (contrainte physique)

Cette approche hybride assure un démarrage rapide tout en maintenant une grande précision sur le long terme.

### 3. Vérification de la fiabilité du modèle
Avant d'utiliser le modèle appris, l'algorithme vérifie sa **fiabilité** :
- Au moins 6 cycles d'apprentissage réussis
- τ (constante de temps) dans une plage plausible (10 à 2000 minutes)
- b stable (coefficient de variation < 35%)

Si ces critères ne sont pas remplis, des **gains "safe"** conservateurs sont utilisés (Kp=0.55, Ki calculé).

### 4. Calcul des Gains (Heuristique)
Une fois le modèle fiable, les gains sont calculés via une heuristique simple :
- **τ** : Constante de temps thermique = 1/b
- **Kp** = 0.35 + 0.9 × sqrt(τ / 200), borné entre 0.10 et 2.50
- **Ki** = Kp / max(τ, 10), borné entre 0.001 et 0.050

### 5. Application de la commande
Enfin, il calcule la puissance à envoyer au radiateur :
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)** : La puissance nécessaire pour compenser les pertes thermiques. Ce terme monte progressivement au démarrage (rampe de warmup) et est capé à 30% tant que le modèle n'est pas fiable.
- **$u_{pi}$ (Correction)** : Le surplus pour corriger l'écart actuel par rapport à la consigne. Utilise une erreur pondérée (2-DOF) pour l'action proportionnelle.

### Anti-windup : Conditional Integration
L'algorithme utilise une technique d'anti-windup appelée **conditional integration** :
- Si la sortie est **saturée à 1.0** ET que l'erreur est **positive** → l'intégrale n'est PAS mise à jour (I:SKIP)
- Si la sortie est **saturée à 0.0** ET que l'erreur est **négative** → l'intégrale n'est PAS mise à jour (I:SKIP)
- Sinon → intégration normale (I:RUN)

Cela évite l'accumulation excessive d'erreur intégrale ("windup") quand l'actionneur ne peut pas appliquer plus de puissance.

### Integrator Hold (temps mort)
Si l'intégrateur est en mode **HOLD** (pendant les périodes de "dead time" après une commutation), l'intégrale est gelée pour éviter le pompage.

### Sign-Flip Leak (décharge douce)
Quand l'erreur change de signe (température passe au-dessus ou en-dessous de la consigne), l'intégrale est partiellement déchargée (30% par cycle pendant 2 cycles). Cela permet un "atterrissage" plus doux et évite les oscillations autour de la consigne.

### Near-Band Scheduling (réduction des gains)
Dans une bande de ±0.5°C autour de la consigne, les gains Kp et Ki sont réduits (×0.60 et ×0.85 respectivement) pour un comportement plus doux à l'approche de la cible et éviter de coller au deadband.

### 2-DOF PI (pondération de consigne)
L'action proportionnelle utilise une erreur pondérée `e_p = 0.4 × erreur` (au lieu de `erreur` directement), ce qui réduit la réactivité sur les changements de consigne tout en préservant le signe de l'erreur.

### Protection contre le dépassement
La vitesse de changement de puissance est limitée (25% par minute) pour éviter les à-coups.

### Filtre de consigne asymétrique
Lorsque vous changez la consigne (ex: de 19°C à 21°C), l'algorithme peut appliquer un filtre EMA asymétrique (si activé via "Utiliser le filtre de consigne"). Ce filtre offre un **atterrissage en douceur** tout en conservant une **montée rapide** :

**Activation conditionnelle** :
- Le filtre ne s'active que lorsque le modèle thermique est **fiable** (tau_reliable = true). Pendant la phase d'apprentissage, le filtre est désactivé pour permettre des réponses complètes (100% de puissance, overshoots) nécessaires à l'estimation des paramètres `a` et `b`.

**Activation au point médian** :
- Lors d'une augmentation de consigne, le filtre ne s'active qu'à partir du **milieu** de l'écart entre la température initiale et la consigne
- Exemple : température actuelle 18°C, consigne 22°C → point médian = 20°C
  - De 18°C à 20°C : **Pleine puissance**, montée rapide
  - De 20°C à 22°C : **Filtre actif**, atterrissage progressif

**Comportement asymétrique** :
- **Augmentation de consigne (HEAT)** : Filtrage EMA progressif après le point médian
- **Baisse de consigne (HEAT)** : Suivi instantané pour économiser l'énergie

Ce comportement est inversé en mode COOL.

## Métriques de diagnostic

L'algorithme expose plusieurs métriques dans les attributs de l'entité climate :

| Métrique | Description |
|----------|-------------|
| **a** | Efficacité du chauffage (°C/min à 100% de puissance) |
| **b** | Coefficient de déperdition (1/min) |
| **tau_min** | Constante de temps thermique de la pièce (en minutes) |
| **tau_reliable** | Indique si le modèle thermique est considéré fiable |
| **learn_ok_count** | Nombre total de cycles d'apprentissage réussis |
| **learn_ok_count_a** | Nombre de cycles où `a` a été appris (phases ON) |
| **learn_ok_count_b** | Nombre de cycles où `b` a été appris (phases OFF) |
| **learn_last_reason** | Raison du dernier résultat d'apprentissage |
| **Kp**, **Ki** | Gains du régulateur PI (calculés ou "safe") |
| **u_ff** | Composante feed-forward de la commande |
| **i_mode** | État de l'intégrateur (I:RUN, I:SKIP, I:HOLD, I:LEAK) |
| **sat** | État de saturation (NO_SAT, SAT_HI, SAT_LO) |
| **error** | Écart entre la consigne et la température actuelle |
| **error_p** | Erreur pondérée 2-DOF utilisée pour l'action proportionnelle |
| **error_filtered** | Erreur filtrée par EMA (lissage des capteurs quantifiés) |
| **integral_error** | Erreur intégrale accumulée |
| **cycles_since_reset** | Nombre de cycles depuis le dernier reset |
| **sign_flip_leak_left** | Cycles restants de décharge d'intégrale |
| **filtered_setpoint** | Consigne interne après filtre asymétrique (pour le debug) |
| **skip_learning_cycles_left** | Nombre de cycles d'apprentissage restants à ignorer après une reprise |

La métrique `tau_reliable` devient `true` quand le modèle a accumulé suffisamment de données fiables (au moins 6 apprentissages réussis, τ dans la plage 10-2000 minutes, et b stable).

### Gestion des perturbations (fenêtres, etc.)

Lorsqu'une fenêtre est détectée ouverte, le thermostat passe en mode OFF et SmartPI arrête ses calculs. Quand la fenêtre est fermée et que le thermostat reprend en mode HEAT :

- **SmartPI ignore automatiquement les 2 premiers cycles d'apprentissage** après la reprise
- Cela permet à la température de se stabiliser avant de reprendre l'apprentissage
- La métrique `skip_learning_cycles_left` indique le nombre de cycles restants à ignorer
- La raison `learn_last_reason` affiche "skip:resume(N left)" pendant cette période

Ce comportement évite que le modèle thermique soit faussé par les données de température perturbées pendant l'ouverture de la fenêtre.

## Conseils pour un apprentissage optimal

Pour que l'algorithme apprenne efficacement le comportement thermique de votre pièce, suivez ces recommandations :

### ✅ À faire

- **Fixer une consigne stable** pendant 2-3 jours minimum
- **Consigne modérée** : 1-2°C au-dessus de la température actuelle (génère des cycles partiels, plus informatifs)
- **Laisser le système en mode HEAT** (l'algo n'apprend qu'en chauffage)
- **Attendre les cycles jour/nuit** (les variations de température extérieure aident à apprendre les pertes thermiques)

### ❌ À éviter

- **Changer la consigne fréquemment** : crée des transitoires qui perturbent l'apprentissage
- **Consigne trop haute** : provoque des cycles à 100% de puissance, moins informatifs
- **Ouvrir les fenêtres** : perturbe le modèle thermique en cours d'apprentissage
- **Couper le chauffage manuellement** : interrompt la collecte de données

### Procédure recommandée

1. Activer SmartPI et vérifier que le thermostat est en mode HEAT
2. Fixer la consigne à ~1-2°C au-dessus de la température actuelle
3. Laisser fonctionner pendant 2-3 jours sans modifier la consigne
4. Vérifier dans les attributs que `learn_ok_count` > 6
5. La métrique `tau_reliable` devrait passer à `true`

> **Note** : L'algorithme continue d'apprendre en permanence. La phase initiale établit une base, puis il s'affine continuellement pour s'adapter aux changements (météo, isolation, etc.).

## Cas d'utilisation recommandés

SmartPI est particulièrement adapté pour :

- 🏠 Les installations où vous ne connaissez pas les bons coefficients TPI
- 🔄 Les pièces dont les caractéristiques thermiques changent (exposition soleil, occupation variable)
- ⚡ Les utilisateurs qui veulent une solution "plug and play"

## Différences avec TPI et Auto-TPI

| Aspect | TPI | Auto-TPI | SmartPI |
|--------|-----|----------|--------|
| **Coefficients** | Fixes (manuels) | Appris puis fixes | Adaptatifs en continu |
| **Configuration** | Complexe (Kint, Kext) | Moyenne | Simple (2 paramètres) |
| **Temps d'adaptation** | Immédiat | ~50 cycles | Continu |
| **Apprentissage** | Aucun | Phase finie | Permanent |
| **Type de modèle** | Proportionnel simple | Observation statistique | Modèle thermique (Theil-Sen) |
| **Méthode de tuning** | Manuel | Heuristique | Heuristique adaptative |

> **Note** : SmartPI est une approche complémentaire à TPI et Auto-TPI. Il convient particulièrement aux utilisateurs qui préfèrent une solution automatique sans configuration.

## Services

### reset_smart_pi_learning

Ce service permet de réinitialiser complètement l'apprentissage de l'algorithme SmartPI. Toutes les données apprises (modèle thermique, gains du régulateur) sont remises à leurs valeurs par défaut.

Cela peut être utile si :
- Vous avez effectué des travaux d'isolation importants
- Vous avez changé le radiateur
- Le comportement du thermostat ne semble plus optimal après un événement extérieur perturbateur

L'apprentissage recommencera de zéro dès le prochain cycle de chauffe.

```yaml
service: versatile_thermostat.reset_smart_pi_learning
target:
  entity_id: climate.my_thermostat
```
