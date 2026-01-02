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
3. **Adaptation des gains** : Les paramètres du régulateur sont automatiquement ajustés pour optimiser la réponse

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Température ──► Apprentissage ──► Modèle pièce   │
│   Puissance   ──► (Robust Est.) ──► (a, b)         │
│                         │                           │
│                         ▼                           │
│               Calcul des gains (Kp, Ki)             │
│                         │                           │
│                         ▼                           │
│               Commande de puissance (%)             │
└─────────────────────────────────────────────────────┘
```

## Qu'est-ce que ça fait concrètement ?

- **Au démarrage** : AutoPI utilise des valeurs par défaut raisonnables
- **Après quelques heures** : Il commence à comprendre votre installation
- **Après quelques jours** : Le modèle est affiné et la régulation devient optimale

### Avantages
 
 ✅ **Pas de réglage manuel** : Pas besoin de trouver les bons coefficients Kint/Kext  
 ✅ **S'adapte aux changements** : Si vous changez de radiateur ou isolez, il se ré-adapte  
 ✅ **Évite les oscillations** : La bande morte intégrée empêche les régulations inutiles près de la consigne  
 ✅ **Robuste aux perturbations** : L'algo ignore les variations brutales (ex: ouverture de fenêtre) pour ne pas fausser le modèle.
 ✅ **Gestion de l'inertie** : Prend en compte le temps de réaction ("Dead time") de votre chauffage pour éviter de chauffer trop tôt ou trop tard.
 ✅ **Gain Scheduling** : Ajuste la douceur de la régulation en fonction de la distance à la consigne (plus doux à l'approche du but).

## Configuration

Pour activer AutoPI :

1. Créez ou modifiez un VTherm de type **switch** ou **vanne**
2. Dans **Sous-jacents**, sélectionnez l'algorithme **AutoPI**
3. Configurez les paramètres dans le menu **AutoPI**

## Paramètres disponibles

| Paramètre | Description | Valeur par défaut | Recommandation |
|-----------|-------------|-------------------|----------------|
| **Bande morte (°C)** | Zone autour de la consigne où aucune action n'est effectuée. Évite les micro-régulations | 0.05°C | 0.05 - 0.1°C |
| **Agressivité** | Vitesse de réponse du régulateur. Plus bas = plus rapide | 0.5 | 0.3 (rapide) à 1.0 (lent) |

### Réglage de l'agressivité

- **0.1 - 0.3** : Réponse très rapide. Pour des pièces avec faible inertie (petites pièces, chauffage électrique direct)
- **0.5** : Équilibré. Bon point de départ pour la plupart des installations
- **1.0 - 2.0** : Réponse lente. Pour des systèmes à forte inertie (plancher chauffant, radiateurs à eau)

## Fonctionnement détaillé de l'algorithme "Robuste"

Le fonctionnement d'AutoPI peut se découper en 4 étapes cycliques :

### 1. Mesure et Observation
À chaque cycle, l'algorithme collecte :
- La température actuelle ($T_{int}$)
- La température extérieure ($T_{ext}$)
- La puissance qui a été envoyée au radiateur ($u$)

### 2. Modélisation (Robust Estimator)
Il met à jour son modèle interne de la pièce défini par deux paramètres :
- **a** (Efficacité) : Combien de degrés je gagne par minute si je chauffe à 100%.
- **b** (Déperdition) : Combien de degrés je perds par minute par degré d'écart avec l'extérieur.

C'est ici que l'approche "Robuste" intervient en filtrant les anomalies (ex: chute brutale de température) pour ne pas fausser ces paramètres $a$ et $b$.

### 3. Calcul des Gains (PI Tuning)
Une fois qu'il connaît la pièce ($a$, $b$) et son inertie (Dead time), il calcule les coefficients idéaux pour le régulateur PI :
- **Kp** (Proportionnel) : Calculé pour réagir vite mais sans dépasser.
- **Ki** (Intégral) : Calculé pour annuler l'erreur résiduelle.

*Le "Gain Scheduling" réduit ces gains quand on est proche de la consigne pour un atterrissage en douceur.*

### 4. Application de la commande
Enfin, il calcule la puissance à envoyer au radiateur :
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)** : La puissance juste nécessaire pour maintenir la température (basé sur $T_{ext}$ et les déperditions $b$).
- **$u_{pi}$ (Correction)** : Le surplus pour corriger l'écart actuel par rapport à la consigne.

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
| **Type de modèle** | Proportionnel simple | Observation statistique | Modèle thermique (Robust Estimator) |

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
