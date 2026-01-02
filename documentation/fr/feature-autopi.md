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

Cette version d'AutoPI utilise plusieurs mécanismes avancés pour garantir confort et stabilité :

### 1. L'Estimateur Robuste (Le cerveau)
Contrairement aux algorithmes simples qui moyenne bêtement toutes les mesures, AutoPI utilise des statistiques robustes (Médiane et Huber).
- **En clair** : Si vous ouvrez une fenêtre en hiver, la température chute brutalement. AutoPI va détecter que cette chute est anormale ("outlier") et l'ignorer pour ne pas fausser son apprentissage. Il ne retient que le comportement "normal" de la pièce.

### 2. Gestion du "Dead Time" (L'anticipation)
Tous les systèmes de chauffage ont un délai entre le moment où on allume et le moment où ça chauffe vraiment (inertie des résistances, circulation d'eau...).
- **Le problème** : Si on ignore ce délai, le thermostat "s'énerve" et surchauffe car il ne voit pas la température monter tout de suite.
- **La solution** : AutoPI mesure ce délai (`deadtime_s`) et "attend" patiemment avant de corriger, évitant ainsi les oscillations inutiles.

### 3. Gain Scheduling (La douceur)
C'est la capacité à changer de comportement selon la situation :
- **Loin de la consigne** : Le thermostat réagit fort pour monter vite en température.
- **Proche de la consigne** : Il devient très doux pour "atterrir" sur la température cible sans la dépasser.
Cela permet d'avoir à la fois une montée rapide ET une grande stabilité une fois à température.

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
