"""
Smart-PI Deadband Manager.

Manages deadband and near-band state detection with hysteresis,
including auto-calculation of near-band thresholds based on model parameters.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from .const import (
    DEADBAND_ABOVE_C,
    DEADBAND_BELOW_C,
    DEADBAND_HYSTERESIS,
    NEAR_BAND_ABOVE_FACTOR,
    NEAR_BAND_HYSTERESIS_C,
    clamp,
)
from ..vtherm_hvac_mode import VThermHvacMode_COOL

if TYPE_CHECKING:
    from .learning import ABEstimator, DeadTimeEstimator

_LOGGER = logging.getLogger(__name__)


@dataclass
class DeadbandResult:
    """Result of deadband detection."""
    in_deadband: bool
    in_near_band: bool
    deadband_changed: bool  # For bumpless transfer trigger


class DeadbandManager:
    """
    Manages deadband and near-band state detection with hysteresis.

    Features:
    - Asymmetric deadband thresholds for HEAT mode
    - Hysteresis for deadband entry/exit to prevent oscillations
    - Auto-calculation of near-band thresholds based on dead time and model slopes
    - State persistence for restart recovery
    """

    def __init__(self, name: str, near_band_deg: float):
        """
        Initialize DeadbandManager.

        Args:
            name: Entity name for logging
            near_band_deg: Configured near-band threshold (°C)
        """
        self._name = name
        self._near_band_deg = near_band_deg

        # State
        self._in_deadband: bool = False
        self._in_near_band: bool = False

        # Near-band thresholds (auto-calculated or fallback)
        self._near_band_below_deg: float = near_band_deg
        self._near_band_above_deg: float = near_band_deg * NEAR_BAND_ABOVE_FACTOR
        self._near_band_source: str = "init"

    def reset(self) -> None:
        """Reset deadband state to initial values."""
        self._in_deadband = False
        self._in_near_band = False
        self._near_band_below_deg = self._near_band_deg
        self._near_band_above_deg = self._near_band_deg * NEAR_BAND_ABOVE_FACTOR
        self._near_band_source = "reset"

    def update(
        self,
        error: float,
        hvac_mode,  # VThermHvacMode
        tau_reliable: bool,
        dt_est: "DeadTimeEstimator",
        estimator: "ABEstimator",
        current_temp: float,
        ext_temp: Optional[float],
        cycle_min: float,
        deadband_c: float,
    ) -> DeadbandResult:
        """
        Detect deadband and near-band state.

        Args:
            error: Current error (setpoint - temperature) in °C
            hvac_mode: Current HVAC mode (VThermHvacMode)
            tau_reliable: Whether tau (time constant) is reliable
            dt_est: DeadTimeEstimator instance for dead time values
            estimator: ABEstimator instance for model parameters
            current_temp: Current indoor temperature (°C)
            ext_temp: Current outdoor temperature (°C), optional
            cycle_min: Cycle duration in minutes
            deadband_c: Configured deadband for COOL mode (°C)

        Returns:
            DeadbandResult with current state and change flag
        """
        abs_e = abs(error)
        was_in_deadband = self._in_deadband

        # --- Deadband Detection ---
        if not tau_reliable:
            in_deadband_now = False
        else:
            # HEAT mode uses asymmetric deadband, COOL uses symmetric
            if hvac_mode is not None and hvac_mode != VThermHvacMode_COOL:
                # HEAT mode: asymmetric deadband
                db_below = max(DEADBAND_BELOW_C, 0.0)
                db_above = max(DEADBAND_ABOVE_C, 0.0)
                h_below = max(DEADBAND_HYSTERESIS, 0.0)
                h_above = max(DEADBAND_HYSTERESIS, 0.0)
                db_entry = db_below if error >= 0.0 else db_above
                db_exit = (db_below + h_below) if error >= 0.0 else (db_above + h_above)

                if abs_e < db_entry:
                    in_deadband_now = True
                elif abs_e > db_exit:
                    in_deadband_now = False
                else:
                    in_deadband_now = self._in_deadband
            else:
                # COOL mode: symmetric deadband
                db_entry = deadband_c
                db_exit = deadband_c + DEADBAND_HYSTERESIS
                if abs_e < db_entry:
                    in_deadband_now = True
                elif abs_e > db_exit:
                    in_deadband_now = False
                else:
                    in_deadband_now = self._in_deadband

        self._in_deadband = in_deadband_now
        deadband_changed = was_in_deadband and not in_deadband_now

        # --- Near-Band Detection ---
        if not tau_reliable:
            in_near_band_now = False
        else:
            # HEAT mode uses auto near-band calculation, COOL uses static value
            if hvac_mode is not None and hvac_mode != VThermHvacMode_COOL:
                # HEAT mode: update auto near-band if deadtime is reliable
                if dt_est.deadtime_heat_reliable:
                    self.update_near_band_auto(
                        hvac_mode, current_temp, ext_temp, dt_est, estimator, cycle_min
                    )

                nb_below = self._near_band_below_deg
                nb_above = self._near_band_above_deg
                nb_hyst = max(NEAR_BAND_HYSTERESIS_C, 0.0)
                nb_entry = nb_below if error >= 0.0 else nb_above
                nb_exit = nb_entry + nb_hyst

                if abs_e <= nb_entry:
                    in_near_band_now = True
                elif abs_e >= nb_exit:
                    in_near_band_now = False
                else:
                    in_near_band_now = self._in_near_band
            else:
                # COOL mode: simple symmetric near-band
                in_near_band_now = (self._near_band_deg > 0.0) and (abs_e <= self._near_band_deg)

        self._in_near_band = in_near_band_now

        return DeadbandResult(
            in_deadband=in_deadband_now,
            in_near_band=in_near_band_now,
            deadband_changed=deadband_changed,
        )

    def update_near_band_auto(  # pylint: disable=unused-argument
        self,
        hvac_mode,  # VThermHvacMode - reserved for future mode-specific logic
        current_temp: float,
        ext_temp: Optional[float],
        dt_est: "DeadTimeEstimator",
        estimator: "ABEstimator",
        cycle_min: float,
    ) -> None:
        """
        Calculate Near-Band thresholds based on Dead Time and Model Slopes.

        Logic:
        1. If deadtime not reliable, fallback to manual config.
        2. Calculate Horizons (H) based on Dead Time (L) + half cycle.
           - L_cool uses deadtime_cool_s if available/reliable, else defaults to L_heat.
        3. Estimate Slopes from Model (a, b):
           - s_cool = b * (Tin - Text)
           - s_heat_net = a - s_cool
        4. Apply formulas: NB = Slope * H.
        5. Store results in self._near_band_* variables.
        """
        # 1. Fallback Check (Deadtime)
        if not dt_est.deadtime_heat_reliable or dt_est.deadtime_heat_s is None:
            self._near_band_below_deg = self._near_band_deg
            self._near_band_above_deg = self._near_band_deg * NEAR_BAND_ABOVE_FACTOR
            self._near_band_source = "fallback_deadtime"
            return

        # Check External Temp availability for Slope Estimation
        if ext_temp is None:
            self._near_band_below_deg = self._near_band_deg
            self._near_band_above_deg = self._near_band_deg * NEAR_BAND_ABOVE_FACTOR
            self._near_band_source = "fallback_no_ext"
            return

        # Horizons Configuration
        L_heat = dt_est.deadtime_heat_s
        use_cool_deadtime = dt_est.deadtime_cool_reliable and dt_est.deadtime_cool_s is not None
        L_cool = dt_est.deadtime_cool_s if use_cool_deadtime else L_heat

        cycle_s = max(cycle_min * 60.0, 60.0)  # Safety

        # Horizon H = L + delta (delta = half cycle delay approx)
        H_below = L_cool + (cycle_s / 2.0)
        H_above = L_heat + (cycle_s / 2.0)

        # 2. Model-based Slope Estimation
        # Check basic model reliability
        # We need positive 'a' and 'b'.
        if estimator.learn_ok_count_a < 10 or estimator.a <= 1e-6:
            self._near_band_below_deg = self._near_band_deg
            self._near_band_above_deg = self._near_band_deg * NEAR_BAND_ABOVE_FACTOR
            self._near_band_source = "fallback_model"
            return

        # Calculate slopes based on current conditions
        # s_cool (deg/min) = b * (Tin - Text).
        # This is the natural temperature drop rate.
        delta_T = current_temp - ext_temp
        # Clamp delta_T to avoid negative cooling slope in weird cases
        # (e.g. Text > Tin in winter?)
        # For heating mode logic, we assume Tin > Text usually.
        # If Tin < Text, s_cool would be negative (gain), which confuses the logic.
        # We assume s_cool >= 0 (loss).
        s_cool = estimator.b * max(delta_T, 0.0)

        # s_heat_net (deg/min) = a - s_cool
        # This is the net temperature rise rate at 100% power.
        s_heat_net = estimator.a - s_cool

        # Safety: if s_heat_net is too small, fallback
        if s_heat_net <= 1e-5:
            self._near_band_below_deg = self._near_band_deg
            self._near_band_above_deg = self._near_band_deg * NEAR_BAND_ABOVE_FACTOR
            self._near_band_source = "fallback_slope_low"
            return

        # 3. Asymmetry Factor
        # alpha = s_cool / (s_heat_net + eps) clamped [0.3, 1.0]
        alpha = clamp(s_cool / s_heat_net, 0.3, 1.0)

        # 4. Calculate Raw Bands
        # Convert slopes to deg/sec for H multiplication
        s_heat_s = s_heat_net / 60.0

        # Deadbands (Manual Config) - Base
        db_below = max(DEADBAND_BELOW_C, 0.0)
        db_above = max(DEADBAND_ABOVE_C, 0.0)

        # Formula: NB = DB + Slope*H
        nb_below_raw = db_below + (1.0 * s_heat_s * H_below)
        nb_above_raw = db_above + (alpha * s_heat_s * H_above)

        # 5. Apply Constraints
        # NB_below >= DB_below + 0.1
        self._near_band_below_deg = clamp(nb_below_raw, db_below + 0.1, 2.0)

        # NB_above >= DB_above + 0.1 AND <= NB_below
        nb_above_constrained = clamp(nb_above_raw, db_above + 0.1, self._near_band_below_deg)
        self._near_band_above_deg = nb_above_constrained

        _LOGGER.debug(
            "%s - nearband auto: using deadtime_cool=%ss (fallback=%s) -> nb_above=%s",
            self._name,
            f"{L_cool:.1f}",
            not use_cool_deadtime,
            f"{self._near_band_above_deg:.3f}",
        )

        self._near_band_source = "auto_model_aware"

    def load_state(self, state: dict) -> None:
        """
        Load state from persistence dict.

        Args:
            state: Dictionary with persisted state
        """
        if not state:
            return

        self._in_deadband = bool(state.get("in_deadband", False))
        self._in_near_band = bool(state.get("in_near_band", False))

        if "near_band_below_deg_auto" in state and state["near_band_below_deg_auto"] is not None:
            self._near_band_below_deg = float(state["near_band_below_deg_auto"])
        if "near_band_above_deg_auto" in state and state["near_band_above_deg_auto"] is not None:
            self._near_band_above_deg = float(state["near_band_above_deg_auto"])
        if "near_band_source" in state and state["near_band_source"] is not None:
            self._near_band_source = str(state["near_band_source"])

    def save_state(self) -> dict:
        """
        Save state to persistence dict.

        Returns:
            Dictionary with current state for persistence
        """
        return {
            "in_deadband": self._in_deadband,
            "in_near_band": self._in_near_band,
            "near_band_below_deg_auto": self._near_band_below_deg,
            "near_band_above_deg_auto": self._near_band_above_deg,
            "near_band_source": self._near_band_source,
        }

    # --- Properties for diagnostic access ---

    @property
    def in_deadband(self) -> bool:
        """Whether currently in deadband."""
        return self._in_deadband

    @in_deadband.setter
    def in_deadband(self, value: bool) -> None:
        self._in_deadband = value

    @property
    def in_near_band(self) -> bool:
        """Whether currently in near-band."""
        return self._in_near_band

    @in_near_band.setter
    def in_near_band(self, value: bool) -> None:
        self._in_near_band = value

    @property
    def near_band_below_deg(self) -> float:
        """Near-band threshold below setpoint (°C)."""
        return self._near_band_below_deg

    @property
    def near_band_above_deg(self) -> float:
        """Near-band threshold above setpoint (°C)."""
        return self._near_band_above_deg

    @property
    def near_band_source(self) -> str:
        """Source of near-band thresholds (e.g., 'auto_model_aware', 'fallback_deadtime')."""
        return self._near_band_source

    @property
    def near_band_deg(self) -> float:
        """Configured near-band threshold (°C)."""
        return self._near_band_deg

    @near_band_deg.setter
    def near_band_deg(self, value: float) -> None:
        """Set configured near-band threshold (°C)."""
        self._near_band_deg = value
