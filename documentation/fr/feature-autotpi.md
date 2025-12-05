# Fonctionnalité Auto TPI

> **Note**: Cette fonctionnalité est disponible à partir de la version xxx

## Introduction

La fonctionnalité **Auto TPI** (ou auto-apprentissage) est une avancée majeure du Versatile Thermostat. Elle permet au thermostat d'ajuster **automatiquement** ses coefficients de régulation (Kp et Ki) en analysant le comportement thermique de votre pièce.

En mode TPI (Time Proportional & Integral), le thermostat calcule un pourcentage d'ouverture ou de temps de chauffe en fonction de l'écart entre la température de consigne et la température intérieure (`Kp`), et de l'influence de la température extérieure (`Ki`).

Trouver les bons coefficients (`tpi_coef_int` et `tpi_coef_ext`) est souvent complexe et nécessite de nombreux essais. **Auto TPI le fait pour vous.**

## Pré-requis

Pour que l'Auto TPI fonctionne efficacement :
1.  **Capteur de température fiable** : Le capteur ne doit pas être influencé directement par la source de chaleur (pas posé sur le radiateur !).
2.  **Capteur de température extérieure** : Une mesure précise de la température extérieure est indispensable.
3.  **Mode TPI activé** : Cette fonctionnalité ne s'applique que si vous utilisez l'algorithme TPI (thermostat sur switch, vanne, ou climate en mode TPI).
4.  **Configuration correcte de la puissance** : Définissez correctement les paramètres liés au temps de chauffe (voir ci-dessous).

## Configuration

La configuration se fait via l'interface d'intégration de Home Assistant (Configurer -> Modifier -> Auto TPI & Learning).

### Paramètres Principaux

| Paramètre | Description |
|---|---|
| **Activer l'Auto TPI** | Cochez cette case pour activer l'apprentissage. |
| **Méthode de calcul** | Choisissez entre `Moyenne` (Average) ou `EMA` (Moyenne Mobile Exponentielle). **Recommandé : EMA**. |
| **Plafond Coefficient Intérieur** | Valeur maximale autorisée pour le coefficient intérieur (`Kp`). Défaut : 0.6. Évite les oscillations trop fortes. |
| **Plafond Coefficient Extérieur** | Valeur maximale pour le coefficient extérieur (`Ki`). Défaut : 0.04. |

### Configuration Thermique (Critique)

L'algorithme a besoin de comprendre la réactivité de votre système de chauffage.

#### `heater_heating_time` (Temps de réponse thermique)
C'est le temps total nécessaire pour que le système commence à avoir un effet mesurable sur la température ambiante.

Il doit inclure :
*   Le temps de chauffe du radiateur (inertie matérielle).
*   Le temps de propagation de la chaleur dans la pièce jusqu'au capteur.

**Valeurs suggérées :**

| Type de chauffage | Valeur suggérée |
|---|---|
| Radiateur électrique (convecteur), capteur proche | 2-5 min |
| Radiateur à inertie (bain d'huile, fonte), capteur proche | 5-10 min |
| Chauffage au sol, ou grande pièce avec capteur éloigné | 10-20 min |

> Une valeur incorrecte peut fausser le calcul de l'efficacité et empêcher l'apprentissage.

#### `heater_cooling_time` (Temps de refroidissement)
Temps nécessaire pour que le radiateur devienne froid après l'arrêt. Utilisé pour estimer si le radiateur est "chaud" ou "froid" au début d'un cycle.

### Configuration des Capacités de Chauffe/Refroidissement

L'algorithme utilise la notion de **Capacité** (°C gagnés par heure à 100% de puissance) plutôt que de simples taux arbitraires.

*   **Mode Automatique (Recommandé)** : Laissez l'option `use_capacity_as_rate` activée (si disponible) ou laissez le système détecter automatiquement la capacité maximale lors des cycles à 100% de puissance.
*   **Mode Manuel** : Vous pouvez définir manuellement :
    *   `auto_tpi_heating_rate` : Capacité de chauffe en °C/h (ex: 1.0 = le chauffage à fond gagne 1°C par heure).
    *   `auto_tpi_cooling_rate` : Capacité de refroidissement en °C/h.

## Fonctionnement

L'Auto TPI fonctionne de manière cyclique :

1.  **Observation** : À chaque cycle (ex: toutes les 10 min), le thermostat mesure la température au début et à la fin, ainsi que la puissance utilisée.
2.  **Validation** : Il vérifie si le cycle est valide pour l'apprentissage :
    *   La puissance n'était pas saturée (entre 0% et 100% exclu).
    *   L'écart de température est significatif.
    *   Le système est stable (pas d'échecs consécutifs).
3.  **Calcul (Apprentissage)** :
    *   **Priorité 1 : Coefficient Intérieur (`Kp`)**. Si la température a évolué dans le bon sens, il calcule le ratio entre l'évolution réelle et l'évolution théorique attendue. Il ajuste `Kp` pour réduire l'écart.
    *   **Priorité 2 : Coefficient Extérieur (`Ki`)**. Si l'apprentissage intérieur n'est pas possible (ex: température stable mais écart avec la consigne), il ajuste `Ki` pour compenser les pertes thermiques liées à l'extérieur.
4.  **Mise à jour** : Les nouveaux coefficients sont lissés (selon la méthode choisie) et sauvegardés. Ils sont immédiatement utilisés pour le cycle suivant.

## Attributs et Capteurs

Un capteur dédié `sensor.<nom_thermostat>_auto_tpi_learning_state` permet de suivre l'état de l'apprentissage.

**Attributs disponibles :**

*   `active` : L'apprentissage est activé.
*   `heating_cycles_count` : Nombre total de cycles observés.
*   `coeff_int_cycles` : Nombre de fois où le coefficient intérieur a été ajusté.
*   `coeff_ext_cycles` : Nombre de fois où le coefficient extérieur a été ajusté.
*   `model_confidence` : Indice de confiance (0 à 100%) sur la qualité des réglages.
*   `last_learning_status` : Raison du dernier succès ou échec (ex: `learned_indoor_heat`, `power_out_of_range`).
*   `calculated_coef_int` / `calculated_coef_ext` : Valeurs actuelles des coefficients.

## Services

### Réinitialiser l'apprentissage

Si vous changez de radiateur ou déplacez le capteur, il est conseillé de réinitialiser l'apprentissage.

Utilisez le service `versatile_thermostat.set_auto_tpi_mode` :
```yaml
service: versatile_thermostat.set_auto_tpi_mode
target:
  entity_id: climate.mon_thermostat
data:
  enable: true
  reset: true # Force la réinitialisation des compteurs et des coefficients par défaut
```

Ou via l'interface des outils de développement.