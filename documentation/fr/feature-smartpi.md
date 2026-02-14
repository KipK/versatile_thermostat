# L'algorithme SmartPI

- [L'algorithme SmartPI](#lalgorithme-smartpi)
  - [Principe de fonctionnement](#principe-de-fonctionnement)
  - [Phases de fonctionnement](#phases-de-fonctionnement)
  - [Fonctionnalités Avancées](#fonctionnalités-avancées)
  - [Configuration](#configuration)
  - [Métriques de diagnostic](#métriques-de-diagnostic)
  - [Services](#services)

## Principe de fonctionnement

L'algorithme **SmartPI** est un régulateur adaptatif qui apprend automatiquement le comportement thermique de votre pièce. Il est conçu pour remplacer le casse-tête du réglage manuel des coefficients PID/TPI par une approche auto-apprenante.

### Comment ça marche ?

1.  **Apprentissage continu (Heartbeat)** : SmartPI analyse la réponse de la pièce en continu (toutes les minutes) via une fenêtre glissante, sans attendre la fin des cycles de chauffe.
2.  **Modélisation thermique** : Il construit un modèle interne robuste (utilisant une méthode statistique Médiane + MAD) caractérisé par :
    *   **a** (Efficacité) : Gain de température par minute à 100% de puissance.
    *   **b** (Déperdition) : Perte de température par minute et par degré d'écart avec l'extérieur.
    *   **L** (Temps mort) : Délai entre l'allumage du radiateur et le début de chauffage réel.
3.  **Adaptation des gains** : Les coefficients (Kp, Ki) sont recalculés dynamiquement en fonction de l'inertie de la pièce (Tau) et du temps mort détecté.

## Phases de fonctionnement

### Phase 1 : Hystérésis (Bootstrap et Apprentissage initial)

Au tout premier démarrage (ou après un reset de l'apprentissage), le modèle thermique est vide. Pour garantir un confort immédiat tout en générant des données d'apprentissage de qualité, SmartPI commence **obligatoirement** par une phase de **Bootstrap** en mode **Hystérésis** :

*   **ON** : Quand la température passe sous `Consigne - 0.3°C`.
*   **OFF** : Quand la température dépasse `Consigne + 0.5°C`.
*   **Maintien** : Entre les deux seuils, l'état précédent est conservé.

Cette phase génère des cycles de chauffe francs et nets, essentiels pour identifier les paramètres `a` et `b` mais aussi et surtout pour apprendre le **Temps Mort** (Dead Time) initial.

> **Transition** : L'algorithme passe automatiquement en phase **STABLE** dès qu'il a collecté assez de mesures fiables (31 mesures minimum).
> **Note** : En mode Hystérésis, la coupure est **instantanée** dès que la température dépasse le seuil haut, interrompant le cycle PWM en cours pour éviter toute surchauffe.

### Phase 2 : Stable (Régulation PI adaptative)

Une fois le modèle fiable, SmartPI active son régulateur PI avancé :

*   **Feed-Forward (Prédiction)** : Calcule la puissance de base nécessaire pour compenser les pertes thermiques (basé sur la température extérieure).
*   **PI (Correction)** : Ajoute ou retire de la puissance pour corriger l'écart exact avec la consigne.
*   **Raffinage continu** : L'algorithme continue d'affiner son modèle en permanence pour s'adapter aux changements de saison ou d'isolation (via estimation robuste Médiane/MAD).

### Phase 3 : Calibration Forcée (Maintenance du modèle)

Si l'algorithme détecte que ses données de **Temps Mort** ne sont plus fiables ou si aucune calibration n'a eu lieu depuis plus de 72h, il peut déclencher une phase de **Calibration Forcée**.

*   Le thermostat repasse temporairement en mode hystérésis pour effectuer un cycle complet (Refroidissement -> Chauffe -> Refroidissement).
*   Cela permet de recalibrer précisément les délais de réaction du système.
*   Cette phase peut aussi être déclenchée manuellement via un service.

## Fonctionnalités Avancées

SmartPI introduit plusieurs raffinements pour améliorer la stabilité et le confort :

### 1. Estimation du Temps Mort (Dead Time)
SmartPI détecte automatiquement le délai (**L**) entre l'ordre d'allumage et la réaction effective de la température.
*   Cela permet d'utiliser des règles de réglage plus fines (IMC - Internal Model Control) pour éviter les oscillations sur les systèmes à retard (ex: planchers chauffants, bains d'huile).
*   La détection est active même en mode **Hystérésis** (sur les oscillations naturelles).

### 2. Auto-adaptation de la Bande Proche (Auto Near-Band)
Pour éviter les dépassements (overshoot), SmartPI réduit ses gains lorsqu'il approche de la consigne.
*   Cette "zone de douceur" est calculée automatiquement en fonction de l'inertie et du temps mort de la pièce.
*   En mode Chauffage, cette zone est asymétrique : elle commence plus tôt "sous" la consigne pour atterrir en douceur, et serre plus fort "au-dessus" pour couper vite en cas de dépassement.

### 3. Boost à la reprise (Setpoint Boost)
Si vous augmentez la consigne de plus de **0.3°C** (ex: passage de mode Eco à Confort), SmartPI active temporairement un mode "Boost" :
*   Le limiteur de vitesse (rate-limiter) est relâché pour permettre une montée en puissance rapide.
*   L'action proportionnelle est rendue plus agressive pour atteindre la cible au plus vite.

### 4. Filtre de Consigne Asymétrique (Soft Landing)
Pour éviter de dépasser la cible lors d'une montée en température, SmartPI applique un filtre intelligent sur la consigne interne :
*   **Montée** : La consigne interne grimpe progressivement une fois passée la mi-course, forçant le régulateur à ralentir avant l'impact.
*   **Descente** : La consigne est suivie instantanément pour couper le chauffage sans délai (économie d'énergie).

### 5. Protection Thermique (Thermal Guard)
Si vous baissez la consigne (ex: passage Confort à Eco), une "garde thermique" s'active :
*   Elle empêche l'intégrale (la mémoire des erreurs passées) de continuer à monter même si la température est encore sous l'ancienne consigne.
*   Cela évite de stocker de la "chaleur virtuelle" qui provoquerait un dépassement une fois la nouvelle consigne atteinte.

## Configuration

Les paramètres par défaut conviennent à la majorité des cas.

| Paramètre | Description | Valeur conseillée |
|-----------|-------------|-------------------|
| **Bande morte** | Zone de tolérance autour de la consigne (±X°C). | 0.05°C |
| **Filtre de consigne** | Active le "Soft Landing". | Désactivé |

> **Astuce** : Si la température oscille trop, essayez d'augmenter la bande morte.

## Métriques de diagnostic

Pour les utilisateurs avancés, l'entité climate expose des attributs détaillés :

| Attribut | Description |
|----------|-------------|
| `regulation_mode` | Mode actuel : `hysteresis` (apprentissage) ou `smartpi` (régulé) |
| `phase` | Phase actuelle de l'algorithme : `Hysteresis`, `Stable` ou `Calibration` |
| `hysteresis_state`| État en phase hystérésis : `on`, `off` ou `band` |
| `tau_min` | Inertie thermique de la pièce (minutes). Ex: 600 = 10h |
| `tau_reliable` | `true` si l'estimation de l'inertie est fiable |
| `a` | Efficacité de chauffage (°C/min à 100%) |
| `b` | Coefficient de perte (1/min) |
| `learn_ok_count` | Nombre total d'apprentissages validés |
| `learn_ok_count_a` | Nombre d'apprentissages validés pour le paramètre `a` |
| `learn_ok_count_b` | Nombre d'apprentissages validés pour le paramètre `b` |
| `learn_last_reason` | Raison de la dernière tentative d'apprentissage (succès ou motif de rejet) |
| `error` | Écart Consigne - Température |
| `u_ff` | Part de puissance "Feed-Forward" (anticipation météo) |
| `ff_raw` | Puissance brute Feed-Forward avant mise à l'échelle (0.0 à 1.0) |
| `ff_reason` | Raison de l'état/échelle actuel du Feed-Forward |
| `ff_scale` | Facteur d'échelle dynamique pour le Feed-Forward (0.0=off, 1.0=full) |
| `ff_H_inertia_s` | Durée tampon d'inertie pour le lissage FF (secondes) |
| `ff_d_inertia_deg` | Delta température tampon d'inertie pour le lissage FF (°C) |
| `u_pi` | Part de puissance "PI" (correction d'erreur) |
| `Kp`, `Ki` | Gains calculés du régulateur |
| `kp_source` | Origine du gain Kp : `imc_deadtime`, `heuristic`, `safe`, `frozen`, etc. |
| `on_percent` | Puissance totale de consigne (0.0 à 1.0) |
| `u_applied` | Puissance réellement appliquée après toutes limitations |
| `in_deadband` | `true` si la température est dans la zone de confort (Deadband) |
| `in_near_band` | `true` si le système est dans la zone de ralentissement (Near-Band) |
| `near_band_below_deg` | Largeur de la Near-Band sous la consigne (°C, auto-calculée) |
| `near_band_above_deg` | Largeur de la Near-Band au-dessus de la consigne (°C, auto-calculée) |
| `near_band_source` | Origine du calcul Near-Band : `auto_model_aware`, `manual`, etc. |
| `setpoint_boost_active` | `true` si le mode Boost est activé |
| `deadtime_heat_s` | Temps mort estimé en secondes (délai de réaction chauffage) |
| `deadtime_heat_reliable` | `true` si le temps mort de chauffage a été correctement identifié |
| `deadtime_cool_s` | Temps mort estimé en secondes (délai de réaction refroidissement) |
| `deadtime_cool_reliable` | `true` si le temps mort de refroidissement a été correctement identifié |
| `in_deadtime_window` | `true` si le système est actuellement dans une fenêtre de temps mort |
| `governance_regime` | Régime physique détecté (Gouvernance) |
| `governance_cycle_regimes` | Liste des régimes traversés durant le cycle en cours |
| `freeze_reason_thermal` | Raison du gel de l'apprentissage des paramètres thermiques (a, b) |
| `freeze_reason_gains` | Raison du gel de l'adaptation des gains (Kp, Ki) |
| `last_decision_thermal` | Décision de gouvernance pour l'apprentissage thermique |
| `last_decision_gains` | Décision de gouvernance pour l'adaptation des gains |
| `calibration_state` | État actuel de la calibration : `Idle`, `CoolDown`, `HeatUp`, `CoolDownFinal` |
| `last_calibration_time` | Horodatage de la dernière calibration réussie |
| `calibration_retry_count` | Nombre de tentatives de calibration |


## Services

### `reset_smart_pi_learning`

Utilisez ce service si vous changez de radiateur ou d'isolation. Il remet à zéro tous les paramètres appris (`a`, `b`, `deadtime`, etc.) et force un retour en phase **Bootstrap / Hystérésis** pour un nouvel apprentissage propre.

### `force_smart_pi_calibration`

Force le thermostat à entrer immédiatement en phase de **Calibration Forcée**. Utile si vous constatez que la régulation pompe ou si le temps mort affiché semble incorrect.
