"""
Smart-PI Calibration Manager.

Manages forced calibration state machine for dead time estimation.
The calibration cycle drives the system through temperature thresholds
to measure heating and cooling dead times.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .timestamp_utils import convert_monotonic_to_wall_ts, convert_wall_to_monotonic_ts

from .const import (
    HYST_LOWER_C,
    HYST_UPPER_C,
    SmartPICalibrationPhase,
)

if TYPE_CHECKING:
    from .learning import DeadTimeEstimator

_LOGGER = logging.getLogger(__name__)


@dataclass
class CalibrationResult:
    """Result of calibration calculation."""
    on_percent: float | None  # None if not calibrating
    phase: SmartPICalibrationPhase
    message: str = ""


class CalibrationManager:
    """
    Manages forced calibration state machine for dead time estimation.
    
    The calibration cycle consists of three phases:
    1. COOL_DOWN: Drive temperature to low threshold
    2. HEAT_UP: Drive temperature to high threshold (triggers heat deadtime detection)
    3. COOL_DOWN_FINAL: Drive back to low threshold (triggers cool deadtime detection)
    
    After completion, the DeadTimeEstimator should have reliable measurements.
    """

    def __init__(self, name: str):
        """
        Initialize CalibrationManager.
        
        Args:
            name: Entity name for logging
        """
        self._name = name
        
        # State variables
        self._last_calibration_time: float | None = None
        self._calibration_state: SmartPICalibrationPhase = SmartPICalibrationPhase.IDLE
        self._calibration_start_time: float | None = None
        self._force_calibration_requested: bool = False
        self._calibration_retry_count: int = 0

    def reset(self) -> None:
        """Reset calibration state to initial values."""
        self._calibration_state = SmartPICalibrationPhase.IDLE
        self._calibration_start_time = None
        self._force_calibration_requested = False
        self._calibration_retry_count = 0
        # Note: _last_calibration_time is NOT reset to preserve history

    def request_calibration(self) -> None:
        """Request a forced calibration cycle."""
        _LOGGER.info("%s - Manual force calibration requested", self._name)
        self._force_calibration_requested = True
        self._calibration_retry_count = 0  # Reset retries on manual request

    def start_calibration(self, now: float, is_manual: bool = False) -> None:
        """
        Start a calibration cycle.
        
        Args:
            now: Current monotonic timestamp
            is_manual: True if manually requested, False if auto-triggered
        """
        self._calibration_state = SmartPICalibrationPhase.COOL_DOWN
        self._calibration_start_time = now
        self._force_calibration_requested = False
        if is_manual:
            self._calibration_retry_count = 0
        else:
            self._calibration_retry_count += 1

    def handle_timeout(self) -> None:
        """Handle calibration timeout."""
        self._calibration_state = SmartPICalibrationPhase.IDLE
        self._calibration_start_time = None

    def check_and_start(
        self,
        now: float,
        now_wall: float,
        governance_regime,
        phase,
        deadtime_reliable: bool,
        force_calibration_interval_hours: float,
        calibration_retry_max: int,
    ) -> tuple[bool, str]:
        """
        Check if calibration should start and start it if needed.
        
        This consolidates the calibration trigger logic from SmartPI.calculate().
        
        Args:
            now: Current monotonic timestamp
            now_wall: Current wall clock timestamp
            governance_regime: Current governance regime
            phase: Current SmartPI phase
            deadtime_reliable: Whether both heat and cool deadtime are reliable
            force_calibration_interval_hours: Hours between periodic calibrations
            calibration_retry_max: Maximum retry count for auto-triggered calibrations
            
        Returns:
            Tuple of (started, reason) where reason is "manual", "periodic", "missing_deadtime", or "not_started"
        """
        # Already calibrating?
        if self._calibration_state != SmartPICalibrationPhase.IDLE:
            return (False, "already_calibrating")
        
        # Check for manual request
        if self._force_calibration_requested:
            reason = "manual"
            _LOGGER.info("%s - Starting forced calibration (reason=%s)", self._name, reason)
            self.start_calibration(now, is_manual=True)
            return (True, reason)
        
        # Check for periodic calibration
        periodic_due = (
            self._last_calibration_time is not None and
            (now_wall - self._last_calibration_time) > (force_calibration_interval_hours * 3600)
        )
        
        # Check if in stable mode (accept both governance regime and phase)
        # Also accept HYSTERESIS for periodic calibration since system may not have enough samples yet
        # Use string comparison for enums to handle both enum objects and string values
        regime_str = str(governance_regime).upper() if governance_regime is not None else ""
        phase_str = str(phase).upper() if phase is not None else ""
        # For periodic calibration, accept HYSTERESIS as system may be in early learning
        in_stable_for_periodic = "EXCITED_STABLE" in regime_str or "STABLE" in phase_str or "HYSTERESIS" in phase_str
        # For missing_deadtime calibration, only trigger in STABLE phase (not HYSTERESIS)
        # HYSTERESIS is the expected phase during initial learning when deadtime is not yet reliable
        in_stable_for_missing_deadtime = "EXCITED_STABLE" in regime_str or "STABLE" in phase_str
        
        if periodic_due and in_stable_for_periodic:
            reason = "periodic"
            _LOGGER.info("%s - Starting forced calibration (reason=%s)", self._name, reason)
            self.start_calibration(now, is_manual=False)
            return (True, reason)
        
        # Check for missing deadtime - only in STABLE phase, not HYSTERESIS
        if not deadtime_reliable and self._calibration_retry_count < calibration_retry_max and in_stable_for_missing_deadtime:
            reason = "missing_deadtime"
            _LOGGER.info("%s - Starting forced calibration (reason=%s)", self._name, reason)
            self.start_calibration(now, is_manual=False)
            return (True, reason)
        
        return (False, "not_started")

    def should_start_calibration(
        self,
        now: float,
        _governance_regime,  # GovernanceRegime
        _phase,  # SmartPIPhase
        deadtime_reliable: bool,
    ) -> tuple[bool, str]:
        """
        Check if calibration should start.
        
        Args:
            now: Current timestamp
            governance_regime: Current governance regime
            phase: Current SmartPI phase
            deadtime_reliable: Whether deadtime is already reliable
            
        Returns:
            Tuple of (should_start, reason)
        """
        # Check if calibration was requested
        if not self._force_calibration_requested:
            return (False, "not_requested")
        
        # Already calibrating?
        if self._calibration_state != SmartPICalibrationPhase.IDLE:
            return (False, "already_calibrating")
        
        # Start calibration
        _LOGGER.info(
            "%s - Starting forced calibration cycle (deadtime_reliable=%s)",
            self._name,
            deadtime_reliable,
        )
        self._calibration_state = SmartPICalibrationPhase.COOL_DOWN
        self._calibration_start_time = now
        self._force_calibration_requested = False
        return (True, "calibration_requested")

    def calculate(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode,  # VThermHvacMode
        dt_est: "DeadTimeEstimator",
        max_on_percent: float | None,
    ) -> CalibrationResult:
        """
        Execute calibration state machine.
        
        Args:
            target_temp: Target temperature (°C)
            current_temp: Current temperature (°C)
            hvac_mode: Current HVAC mode (VThermHvacMode)
            dt_est: DeadTimeEstimator instance for feeding measurements
            max_on_percent: Maximum on_percent allowed
            
        Returns:
            CalibrationResult with on_percent (or None if not calibrating)
        """
        # If not calibrating, return None
        if self._calibration_state == SmartPICalibrationPhase.IDLE:
            return CalibrationResult(
                on_percent=None,
                phase=self._calibration_state,
                message="idle",
            )
        
        # Safety/Exit conditions - HVAC OFF
        if hvac_mode == VThermHvacMode_OFF:
            self._calibration_state = SmartPICalibrationPhase.IDLE
            return CalibrationResult(
                on_percent=0.0,
                phase=SmartPICalibrationPhase.IDLE,
                message="hvac_off",
            )

        # Determine heating vs cooling mode
        is_cool = (hvac_mode == VThermHvacMode_COOL)
        on_low = 1.0 if is_cool else 0.0
        on_high = 0.0 if is_cool else 1.0
        
        on_percent: float = 0.0
        message: str = ""

        # 1. COOL_DOWN: Drive to Low
        if self._calibration_state == SmartPICalibrationPhase.COOL_DOWN:
            on_percent = on_low
            if current_temp <= target_temp - HYST_LOWER_C:
                _LOGGER.info("%s - Calibration: Reached Low Threshold -> HEAT_UP", self._name)
                self._calibration_state = SmartPICalibrationPhase.HEAT_UP
                # Immediate transition to next state output for better responsiveness
                on_percent = on_high
                message = "transition_to_heat_up"
            else:
                message = "cool_down"

        # 2. HEAT_UP: Drive to High (triggers Heat Deadtime)
        elif self._calibration_state == SmartPICalibrationPhase.HEAT_UP:
            on_percent = on_high
            if current_temp >= target_temp + HYST_UPPER_C:
                _LOGGER.info("%s - Calibration: Reached High Threshold -> COOL_DOWN_FINAL", self._name)
                self._calibration_state = SmartPICalibrationPhase.COOL_DOWN_FINAL
                on_percent = on_low
                message = "transition_to_cool_down_final"
            else:
                message = "heat_up"

        # 3. COOL_DOWN_FINAL: Drive back to Low (triggers Cool Deadtime)
        elif self._calibration_state == SmartPICalibrationPhase.COOL_DOWN_FINAL:
            on_percent = on_low
            if current_temp <= target_temp - HYST_LOWER_C:
                _LOGGER.info("%s - Calibration: Cycle Completed -> IDLE", self._name)
                self._calibration_state = SmartPICalibrationPhase.IDLE
                self._last_calibration_time = time.time()
                self._calibration_start_time = None
                self._calibration_retry_count = 0  # <--- Reset retry count on success
                message = "completed"
                
                # Check success (logging)
                if not dt_est.deadtime_heat_reliable:
                    _LOGGER.warning("%s - Calibration finished but DeadTime still unreliable.", self._name)
            else:
                message = "cool_down_final"

        # Apply max_on_percent limit if specified
        if max_on_percent is not None and on_percent > max_on_percent:
            on_percent = max_on_percent

        # Feed Estimator (Critical for detection!)
        if current_temp is not None:
            dt_est.update(
                now=time.monotonic(),
                tin=current_temp,
                sp=target_temp,
                u_applied=on_percent,
                max_on_percent=max_on_percent if max_on_percent is not None else 1.0,
                is_hysteresis=True,
            )

        return CalibrationResult(
            on_percent=on_percent,
            phase=self._calibration_state,
            message=message,
        )

    def load_state(self, state: dict) -> None:
        """
        Load state from persistence dict.
        
        Args:
            state: Dictionary containing calibration state
        """
        self._last_calibration_time = state.get("last_calibration_time")
        
        # Load calibration state enum
        calibration_state_str = state.get("calibration_state")
        if calibration_state_str:
            try:
                self._calibration_state = SmartPICalibrationPhase(calibration_state_str)
            except ValueError:
                self._calibration_state = SmartPICalibrationPhase.IDLE
        else:
            self._calibration_state = SmartPICalibrationPhase.IDLE
            
        self._calibration_start_time = convert_wall_to_monotonic_ts(state.get("calibration_start_time"))
        self._force_calibration_requested = state.get("force_calibration_requested", False)
        self._calibration_retry_count = state.get("calibration_retry_count", 0)

    def save_state(self) -> dict:
        """
        Save state to persistence dict.
        
        Returns:
            Dictionary containing calibration state
        """
        return {
            "last_calibration_time": self._last_calibration_time,
            "calibration_state": self._calibration_state.value if self._calibration_state else None,
            "calibration_start_time": convert_monotonic_to_wall_ts(self._calibration_start_time),
            "force_calibration_requested": self._force_calibration_requested,
            "calibration_retry_count": self._calibration_retry_count,
        }

    # -------------------------------------------------------------------------
    # Properties for diagnostic access
    # -------------------------------------------------------------------------

    @property
    def state(self) -> SmartPICalibrationPhase:
        """Get current calibration phase."""
        return self._calibration_state

    @property
    def is_calibrating(self) -> bool:
        """Check if calibration is in progress."""
        return self._calibration_state != SmartPICalibrationPhase.IDLE

    @property
    def last_calibration_time(self) -> float | None:
        """Get timestamp of last completed calibration."""
        return self._last_calibration_time

    @last_calibration_time.setter
    def last_calibration_time(self, value: float | None) -> None:
        self._last_calibration_time = value

    @property
    def calibration_start_time(self) -> float | None:
        """Get timestamp of current calibration start."""
        return self._calibration_start_time

    @calibration_start_time.setter
    def calibration_start_time(self, value: float | None) -> None:
        self._calibration_start_time = value

    @property
    def retry_count(self) -> int:
        """Get current retry count."""
        return self._calibration_retry_count

    @retry_count.setter
    def retry_count(self, value: int) -> None:
        self._calibration_retry_count = value

    @property
    def calibration_requested(self) -> bool:
        """Check if calibration has been requested."""
        return self._force_calibration_requested

    @calibration_requested.setter
    def calibration_requested(self, value: bool) -> None:
        self._force_calibration_requested = value


# Import VThermHvacMode at end to avoid circular imports
from ..vtherm_hvac_mode import VThermHvacMode_OFF, VThermHvacMode_COOL  # noqa: E402
