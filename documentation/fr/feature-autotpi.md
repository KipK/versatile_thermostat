# Auto TPI (Apprentissage automatique des coefficients TPI)

Cette fonctionnalité permet au Versatile Thermostat d'apprendre automatiquement les caractéristiques thermiques de votre pièce pour ajuster les coefficients TPI (`tpi_coef_int` et `tpi_coef_ext`) de manière optimale.

## Principe de fonctionnement

L'Auto TPI observe le comportement de votre pièce (température intérieure, extérieure, puissance de chauffage) sur une période donnée pour construire un modèle thermique.

Il utilise un algorithme d'apprentissage basé sur la régression des moindres carrés pondérés pour identifier les paramètres physiques de la pièce.

## Modèle Mathématique

Le modèle repose sur l'équation différentielle thermique suivante :

$$ \frac{dT_{room}}{dt} = -\alpha(T_{room} - T_{ext}) + \beta \cdot Power - \gamma(T_{room} - T_{target}) $$

Où :
*   $\frac{dT_{room}}{dt}$ est la variation de température intérieure (dérivée).
*   $\alpha$ (alpha) est le coefficient de perte thermique (isolation).
*   $\beta$ (beta) est l'efficacité du chauffage (puissance/volume).
*   $\gamma$ (gamma) est un terme de correction lié à l'inertie et à la boucle de contrôle.
*   $Power$ est la puissance de chauffage appliquée (0-100%).

### Algorithme de calcul

1.  **Collecte de données** : Le système enregistre périodiquement les températures et la puissance.
2.  **Filtrage** : Les données aberrantes sont écartées via une méthode IQR (Interquartile Range).
3.  **Régression** : Une régression linéaire (moindres carrés pondérés) est effectuée pour trouver les valeurs optimales de $\alpha$, $\beta$ et $\gamma$ qui minimisent l'erreur entre le modèle et la réalité. Les données récentes ont plus de poids (oubli progressif des anciennes données avec une demi-vie de 7 jours).
4.  **Validation** : La qualité du modèle est évaluée via le coefficient de détermination $R^2$. Si $R^2 < 0.3$, le modèle est jugé insuffisant.

### Calcul des coefficients TPI

Une fois $\alpha$ et $\beta$ déterminés, les coefficients TPI sont calculés ainsi :

*   **Coefficient Extérieur ($K_{ext}$)** :
    $$ K_{ext} = \frac{\alpha}{\beta} $$
    Il représente la puissance nécessaire (en %) pour compenser 1°C de différence avec l'extérieur.
    Valeur bornée entre 0.01 et 0.90.

*   **Coefficient Intérieur ($K_{int}$)** :
    $$ K_{int} = \frac{1}{\beta \times \tau_{target}} $$
    Où $\tau_{target}$ est le temps de réponse souhaité (fixé à 30 minutes). Il représente la réactivité nécessaire pour corriger un écart de température intérieure.
    Valeur bornée entre 0.01 et 0.40.

## Prérequis à l'apprentissage

Pour que l'apprentissage soit validé, il faut :
*   Au moins 100 points de données.
*   Au moins 10 cycles de chauffage complets détectés.
*   Une qualité de modèle ($R^2$) suffisante (au moins "Fair" / > 0.3).

## Indicateurs de qualité

L'état de l'apprentissage est visible via les attributs du thermostat :
*   `learning_quality` : insufficient, poor, fair, good, excellent.
*   `confidence` : Pourcentage de confiance dans le modèle ($R^2 \times 100$).
*   `time_constant` : Constante de temps thermique de la pièce (inertie).
