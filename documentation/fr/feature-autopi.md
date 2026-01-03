# L'algorithme AutoPI

- [L'algorithme AutoPI](#lalgorithme-autopi)
  - [Principe de fonctionnement](#principe-de-fonctionnement)
  - [Qu'est-ce que ça fait concrètement ?](#quest-ce-que-ça-fait-concrètement-)
  - [Configuration](#configuration)
  - [Paramètres disponibles](#paramètres-disponibles)
  - [Cas d'utilisation recommandés](#cas-dutilisation-recommandés)
  - [Différences avec TPI et Auto-TPI](#différences-avec-tpi-et-auto-tpi)

## Principe de fonctionnement

L'algorithme **AutoPI** est un régulateur adaptatif qui apprend automatiquement le comportement thermique de votre pièce. Contrairement à TPI qui utilise des coefficients fixes, AutoPI s'adapte en permanence aux caractéristiques de votre installation.

### Comment ça marche ?

1. **Apprentissage continu** : À chaque cycle de chauffage, AutoPI observe comment la température évolue en fonction de la puissance appliquée
2. **Modélisation thermique** : Il construit un modèle mathématique qui représente votre pièce (inertie, déperditions, puissance du radiateur) via une estimation EWMA conditionnelle
3. **Adaptation des gains** : Les paramètres du régulateur sont automatiquement ajustés via une heuristique basée sur la constante de temps thermique

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Température ──► Apprentissage ──► Modèle pièce   │
│   Puissance   ──► (EWMA cond.)  ──► (a, b)         │
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

- **Au démarrage** : AutoPI utilise des valeurs par défaut conservatrices
- **Après quelques heures** : Il commence à comprendre votre installation
- **Après quelques jours** : Le modèle est affiné et la régulation devient optimale

### Avantages
 
 ✅ **Pas de réglage manuel** : Pas besoin de trouver les bons coefficients Kint/Kext  
 ✅ **S'adapte aux changements** : Si vous changez de radiateur ou isolez, il se ré-adapte  
 ✅ **Évite les oscillations** : Gain scheduling intégré pour une régulation douce près de la consigne  
 ✅ **Robuste aux perturbations** : L'algo ignore les variations brutales pour ne pas fausser le modèle  
 ✅ **Gestion de l'inertie** : Prend en compte la constante de temps thermique de votre pièce  
 ✅ **Anti-dépassement** : Protection par "conditional integration" anti-windup
 ✅ **Fiabilité du modèle** : Utilise des gains "safe" tant que le modèle n'est pas fiable

## Configuration

Pour activer AutoPI :

1. Créez ou modifiez un VTherm de type **switch** ou **vanne**
2. Dans **Sous-jacents**, sélectionnez l'algorithme **AutoPI**
3. Configurez les paramètres dans le menu **AutoPI**

## Paramètres disponibles

| Paramètre | Description | Valeur par défaut | Recommandation |
|-----------|-------------|-------------------|----------------|
| **Bande morte (°C)** | Zone autour de la consigne où aucune action n'est effectuée. Évite les micro-régulations | 0.05°C | 0.05 - 0.1°C |
| **Agressivité** | Facteur de réponse du régulateur. Plus haut = plus doux (gains plus faibles) | 0.5 | 0.3 (réactif) à 1.0 (très doux) |

### Réglage de l'agressivité

L'agressivité influence le calcul heuristique des gains. Une valeur plus haute donne des gains plus faibles et donc une réponse plus lente mais plus stable.

- **0.3** : Réponse réactive, pour des pièces bien isolées avec peu d'inertie
- **0.5** : Équilibre performance/stabilité. Point de départ recommandé
- **1.0** : Réponse très douce, pour éviter les oscillations dans des pièces à forte inertie

## Fonctionnement détaillé de l'algorithme

Le fonctionnement d'AutoPI peut se découper en 5 étapes cycliques :

### 1. Mesure et Observation
À chaque cycle, l'algorithme collecte :
- La température actuelle ($T_{int}$)
- La température extérieure ($T_{ext}$)
- La puissance qui a été envoyée au radiateur ($u$)

### 2. Modélisation (EWMA conditionnelle)
Il met à jour son modèle interne de la pièce défini par deux paramètres :
- **a** (Efficacité) : Combien de degrés je gagne par minute si je chauffe à 100%
- **b** (Déperdition) : Combien de degrés je perds par minute par degré d'écart avec l'extérieur

L'algorithme utilise une estimation EWMA (moyenne mobile exponentielle) **conditionnelle** :
- **b** est appris principalement pendant les phases OFF (u < 5%), quand le refroidissement est observable
- **a** est appris principalement pendant les phases ON (u > 20%), quand le chauffage est significatif

Cette approche évite les interférences entre les deux paramètres et ignore les mesures bruitées.

### 3. Vérification de la fiabilité du modèle
Avant d'utiliser le modèle appris, l'algorithme vérifie sa **fiabilité** :
- Au moins 6 cycles d'apprentissage réussis
- τ (constante de temps) dans une plage plausible (30 à 3000 minutes)
- b stable (coefficient de variation < 35%)

Si ces critères ne sont pas remplis, des **gains "safe"** conservateurs sont utilisés (Kp=1.0, Ki=0.003).

### 4. Calcul des Gains (Heuristique)
Une fois le modèle fiable, les gains sont calculés via une heuristique simple :
- **τ** : Constante de temps thermique = 1/b
- **Kp** = 0.35 + 0.9 × (τ / 200), borné entre 0.2 et 1.5
- **Ki** = Kp / max(τ, 10), borné entre 0.0005 et 0.25

### 5. Application de la commande
Enfin, il calcule la puissance à envoyer au radiateur :
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)** : La puissance nécessaire pour compenser les pertes thermiques. Ce terme est désactivé si le paramètre `a` est trop faible (modèle non fiable).
- **$u_{pi}$ (Correction)** : Le surplus pour corriger l'écart actuel par rapport à la consigne.

### Anti-windup : Conditional Integration
L'algorithme utilise une technique d'anti-windup appelée **conditional integration** :
- Si la sortie est **saturée à 1.0** ET que l'erreur est **positive** → l'intégrale n'est PAS mise à jour (I:SKIP)
- Si la sortie est **saturée à 0.0** ET que l'erreur est **négative** → l'intégrale n'est PAS mise à jour (I:SKIP)
- Sinon → intégration normale (I:RUN)

Cela évite l'accumulation excessive d'erreur intégrale ("windup") quand l'actionneur ne peut pas appliquer plus de puissance.

### Integrator Hold (temps mort)
De plus, si l'intégrateur est en mode **HOLD** (pendant les périodes de "dead time" après une commutation), l'intégrale est gelée pour éviter le pompage.

### Protection contre le dépassement
La vitesse de changement de puissance est limitée (12% par minute par défaut) pour éviter les à-coups.

## Métriques de diagnostic

L'algorithme expose plusieurs métriques dans les attributs de l'entité climate :

| Métrique | Description |
|----------|-------------|
| **a** | Efficacité du chauffage (°C/min à 100% de puissance) |
| **b** | Coefficient de déperdition (1/min) |
| **tau_min** | Constante de temps thermique de la pièce (en minutes) |
| **tau_reliable** | Indique si le modèle thermique est considéré fiable |
| **learn_ok_count** | Nombre de cycles d'apprentissage réussis |
| **learn_last_reason** | Raison du dernier résultat d'apprentissage |
| **Kp**, **Ki** | Gains du régulateur PI (calculés ou "safe") |
| **u_ff** | Composante feed-forward de la commande |
| **i_mode** | État de l'intégrateur (I:RUN, I:SKIP, I:HOLD, I:LEAK) |
| **error** | Écart entre la consigne et la température actuelle |
| **error_filtered** | Erreur filtrée par EMA (lissage des capteurs quantifiés) |
| **integral_error** | Erreur intégrale accumulée |

La métrique `tau_reliable` devient `true` quand le modèle a accumulé suffisamment de données fiables (au moins 6 apprentissages réussis, τ dans une plage plausible, et b stable).

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

1. Activer AutoPI et vérifier que le thermostat est en mode HEAT
2. Fixer la consigne à ~1-2°C au-dessus de la température actuelle
3. Laisser fonctionner pendant 2-3 jours sans modifier la consigne
4. Vérifier dans les attributs que `learn_ok_count` > 6
5. La métrique `tau_reliable` devrait passer à `true`

> **Note** : L'algorithme continue d'apprendre en permanence. La phase initiale établit une base, puis il s'affine continuellement pour s'adapter aux changements (météo, isolation, etc.).

## Cas d'utilisation recommandés

AutoPI est particulièrement adapté pour :

- 🏠 Les installations où vous ne connaissez pas les bons coefficients TPI
- 🔄 Les pièces dont les caractéristiques thermiques changent (exposition soleil, occupation variable)
- ⚡ Les utilisateurs qui veulent une solution "plug and play"

## Différences avec TPI et Auto-TPI

| Aspect | TPI | Auto-TPI | AutoPI |
|--------|-----|----------|--------|
| **Coefficients** | Fixes (manuels) | Appris puis fixes | Adaptatifs en continu |
| **Configuration** | Complexe (Kint, Kext) | Moyenne | Simple (2 paramètres) |
| **Temps d'adaptation** | Immédiat | ~50 cycles | Continu |
| **Apprentissage** | Aucun | Phase finie | Permanent |
| **Type de modèle** | Proportionnel simple | Observation statistique | Modèle thermique (EWMA) |
| **Méthode de tuning** | Manuel | Heuristique | Heuristique adaptative |

> **Note** : AutoPI est une approche complémentaire à TPI et Auto-TPI. Il convient particulièrement aux utilisateurs qui préfèrent une solution automatique sans configuration.

## Services

### reset_auto_pi_learning

Ce service permet de réinitialiser complètement l'apprentissage de l'algorithme AutoPI. Toutes les données apprises (modèle thermique, gains du régulateur) sont remises à leurs valeurs par défaut.

Cela peut être utile si :
- Vous avez effectué des travaux d'isolation importants
- Vous avez changé le radiateur
- Le comportement du thermostat ne semble plus optimal après un événement extérieur perturbateur

L'apprentissage recommencera de zéro dès le prochain cycle de chauffe.

```yaml
service: versatile_thermostat.reset_auto_pi_learning
target:
  entity_id: climate.my_thermostat
```
