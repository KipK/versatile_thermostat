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
│   Puissance   ──►    (RLS)      ──► (a, b)         │
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
