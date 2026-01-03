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
2. **Modélisation thermique** : Il construit un modèle mathématique qui représente votre pièce (inertie, déperditions, puissance du radiateur)
3. **Adaptation des gains** : Les paramètres du régulateur sont automatiquement ajustés selon la méthode SIMC (Skogestad IMC)

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Température ──► Apprentissage ──► Modèle pièce   │
│   Puissance   ──► (Robust Est.) ──► (a, b)         │
│                         │                           │
│                         ▼                           │
│               Calcul des gains (Kp, Ki)             │
│               via méthode SIMC                      │
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
 ✅ **Gestion de l'inertie** : Prend en compte le temps de réaction de votre chauffage via la méthode SIMC  
 ✅ **Anti-dépassement** : Protection multi-niveau contre l'overshoot (feedforward adaptatif, unwinding proportionnel)

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

L'agressivité contrôle la constante de temps en boucle fermée (τc) utilisée dans le calcul SIMC des gains. Une valeur plus haute donne des gains plus faibles et donc une réponse plus lente mais plus stable.

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

### 2. Modélisation (RLS - Recursive Least Squares)
Il met à jour son modèle interne de la pièce défini par deux paramètres :
- **a** (Efficacité) : Combien de degrés je gagne par minute si je chauffe à 100%
- **b** (Déperdition) : Combien de degrés je perds par minute par degré d'écart avec l'extérieur

L'algorithme RLS apprend ces paramètres en continu avec un facteur d'oubli pour s'adapter aux changements de conditions.

### 3. Calcul des Gains (SIMC Tuning)
Une fois qu'il connaît la pièce ($a$, $b$), il calcule les coefficients idéaux pour le régulateur PI selon la méthode **SIMC (Skogestad IMC)** :
- **τ** : Constante de temps thermique = 1/b
- **θ** : Dead time estimé (temps de réaction du chauffage)
- **τc** : Constante de temps en boucle fermée (contrôlée par l'agressivité)
- **Kp** = τ / (a × (τc + θ))
- **Ki** = Kp / min(τ, 4×(τc + θ))

### 4. Gain Scheduling
Quand la température s'approche de la consigne (erreur < 1.5°C), les gains effectifs sont progressivement réduits pour éviter les oscillations :
- À 1.5°C d'écart : 100% des gains
- À 0°C d'écart : 50% des gains

### 5. Application de la commande
Enfin, il calcule la puissance à envoyer au radiateur :
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)** : La puissance nécessaire pour compenser les pertes thermiques. Ce terme est limité pendant la phase d'apprentissage et réduit à zéro en cas de dépassement.
- **$u_{pi}$ (Correction)** : Le surplus pour corriger l'écart actuel par rapport à la consigne.

### Protection contre le dépassement (overshoot)
L'algorithme intègre des protections contre le dépassement de température :
- L'intégrale est réduite proportionnellement à l'amplitude du dépassement
- La vitesse de changement de puissance est limitée, encore plus strictement près de la consigne

## Métriques de diagnostic

L'algorithme expose plusieurs métriques dans les attributs de l'entité climate :

| Métrique | Description |
|----------|-------------|
| **a** | Efficacité du chauffage (°C/min à 100% de puissance) |
| **b** | Coefficient de déperdition (1/min) |
| **tau_min** | Constante de temps thermique de la pièce (en minutes) |
| **confidence_a** | Confiance dans le paramètre a (0-100%) |
| **confidence_b** | Confiance dans le paramètre b (0-100%) |
| **model_confidence** | Confiance globale dans le modèle (moyenne de a et b) |
| **Kp**, **Ki** | Gains de base du régulateur PI calculés par SIMC |
| **effective_Kp**, **effective_Ki** | Gains effectifs après gain scheduling |
| **u_ff** | Composante feed-forward de la commande |
| **error** | Écart entre la consigne et la température actuelle |
| **integral_error** | Erreur intégrale accumulée |

La métrique `model_confidence` est particulièrement utile : une valeur proche de 0% signifie que l'algorithme vient de démarrer, tandis qu'une valeur élevée (>80%) indique que le modèle thermique est bien établi.

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
| **Type de modèle** | Proportionnel simple | Observation statistique | Modèle thermique (RLS) |
| **Méthode de tuning** | Manuel | Heuristique | SIMC (industriel) |

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
