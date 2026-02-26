"""
Constants and Enums for Smart-PI Algorithm.
"""
from enum import Enum
import logging

_LOGGER = logging.getLogger(__name__)

def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into [lo, hi]."""
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x

class SmartPIPhase(str, Enum):
    """Phases of the Smart-PI algorithm."""
    HYSTERESIS = "Hysteresis"  # Learning phase with ON/OFF control
    STABLE = "Stable"          # PI control with reliable model
    CALIBRATION = "Calibration" # Forced calibration cycle in progress

class SmartPICalibrationPhase(str, Enum):
    """Phases of the Smart-PI forced calibration."""
    IDLE = "Idle"
    COOL_DOWN = "CoolDown"
    HEAT_UP = "HeatUp"
    COOL_DOWN_FINAL = "CoolDownFinal"

class SmartPICalibrationResult(str, Enum):
    """Resolution status of a Smart-PI forced calibration cycle."""
    PENDING = "Pending"
    SUCCESS = "Success"
    CANCELLED = "Cancelled"

# ########################################################################
#                      SAFETY-FIRST GOVERNANCE ENUMS                   #
# ########################################################################

class GovernanceRegime(str, Enum):
    """Physical regime detected during a calculation step."""
    WARMUP = "warmup"                  # Hysteresis / bootstrap phase
    EXCITED_STABLE = "excited_stable"  # Normal PI regulation, significant error
    NEAR_BAND = "near_band"            # Close to setpoint, weak signal
    DEAD_BAND = "dead_band"            # In dead band, no action
    HOLD = "hold"                      # Integrator hold active
    PERTURBED = "perturbed"            # External disturbance (window, shedding)
    DEGRADED = "degraded"              # Sensor absent, deadtime unknown
    SATURATED = "saturated"            # Command at 0% or 100%


class FreezeReason(str, Enum):
    """Diagnostic reason why adaptation was frozen."""
    NONE = "none"
    # Structural
    REGIME_TRANSITION = "regime_transition"  # Cycle not homogeneous
    CYCLE_INVALID = "cycle_invalid"
    # Physical / external
    EVENT_POLLUTED = "event_polluted"
    SENSOR_INVALID = "sensor_invalid"
    DEADTIME_UNRELIABLE = "deadtime_unreliable"
    BOOT_GUARD = "boot_guard"
    # Regime-specific
    DEAD_BAND = "dead_band"
    NEAR_BAND = "near_band"
    WARMUP = "warmup"
    HOLD = "hold"
    PERTURBED = "perturbed"
    SATURATION = "saturation"
    SYSTEM_INEFFICIENT = "system_inefficient"


class GovernanceDecision(str, Enum):
    """Decision level for parameter adaptation."""
    ADAPT_ON = "adapt_on"                # Calculation and update allowed
    FREEZE = "freeze"                    # Keep previous values
    HARD_FREEZE = "hard_freeze"          # Absolute prohibition of update
    SOFT_FREEZE_DOWN = "soft_freeze_down" # Only decrease allowed


# Governance matrix: regime -> {domain: (decision, freeze_reason)}
# Domains: 'thermal' (a/b learning), 'gains' (Kp/Ki adaptation)
GOVERNANCE_MATRIX = {
    GovernanceRegime.WARMUP: {
        "thermal": (GovernanceDecision.ADAPT_ON, FreezeReason.NONE),
        "gains": (GovernanceDecision.FREEZE, FreezeReason.WARMUP),
    },
    GovernanceRegime.EXCITED_STABLE: {
        "thermal": (GovernanceDecision.ADAPT_ON, FreezeReason.NONE),
        "gains": (GovernanceDecision.ADAPT_ON, FreezeReason.NONE),
    },
    GovernanceRegime.NEAR_BAND: {
        "thermal": (GovernanceDecision.HARD_FREEZE, FreezeReason.NEAR_BAND),
        "gains": (GovernanceDecision.SOFT_FREEZE_DOWN, FreezeReason.NEAR_BAND),
    },
    GovernanceRegime.DEAD_BAND: {
        "thermal": (GovernanceDecision.HARD_FREEZE, FreezeReason.DEAD_BAND),
        "gains": (GovernanceDecision.HARD_FREEZE, FreezeReason.DEAD_BAND),
    },
    GovernanceRegime.SATURATED: {
        "thermal": (GovernanceDecision.ADAPT_ON, FreezeReason.NONE),  # Allow learning
        "gains": (GovernanceDecision.FREEZE, FreezeReason.SATURATION),  # Keep gains frozen
    },
    GovernanceRegime.HOLD: {
        "thermal": (GovernanceDecision.HARD_FREEZE, FreezeReason.HOLD),
        "gains": (GovernanceDecision.SOFT_FREEZE_DOWN, FreezeReason.HOLD),
    },
    GovernanceRegime.PERTURBED: {
        "thermal": (GovernanceDecision.HARD_FREEZE, FreezeReason.PERTURBED),
        "gains": (GovernanceDecision.HARD_FREEZE, FreezeReason.PERTURBED),
    },
    GovernanceRegime.DEGRADED: {
        "thermal": (GovernanceDecision.HARD_FREEZE, FreezeReason.SENSOR_INVALID),
        "gains": (GovernanceDecision.HARD_FREEZE, FreezeReason.SENSOR_INVALID),
    },
}

# ------------------------------
# Default controller parameters
# ------------------------------

# Safe fallback gains when model is unreliable
KP_SAFE = 0.55
KI_SAFE = 0.010

# Allowed ranges for computed gains
KP_MIN = 0.10
KP_MAX = 5.0
KI_MIN = 0.001
KI_MAX = 0.050


# Anti-windup / integrator behavior
INTEGRAL_LEAK = 0.995  # leak factor per cycle when inside deadband
MAX_STEP_PER_MINUTE = 0.25  # max output change per minute (rate limit)

# Setpoint step boost: faster rate limit when setpoint changes significantly
# This allows quick power ramp-up when user increases setpoint
SETPOINT_BOOST_THRESHOLD = 0.3   # min setpoint change (°C) to trigger boost
SETPOINT_BOOST_ERROR_MIN = 0.3   # min error (°C) to keep boost active
SETPOINT_BOOST_RATE = 0.50       # boosted rate limit (/min) vs 0.15 normal

# Setpoint change handling (mode change vs adjustment)
# - Large change (>= threshold): mode change (eco ↔ comfort) -> reset PI state
# - Small change (< threshold): minor adjustment -> bumpless transfer with limited output jump
SETPOINT_MODE_DELTA_C = 0.5      # °C threshold for mode change detection
SETPOINT_BUMPLESS_MAX_DU = 0.12  # Max allowed output change (0..1) for bumpless transfer
OVERSHOOT_I_CLAMP_EPS_C = 0.10  # Guard band below setpoint where integral cannot increase (°C)

# Tracking anti-windup (back-calculation) tuned for slow thermal systems
AW_TRACK_TAU_S = 120.0        # tracking time constant in seconds (typ. 60-180s)
AW_TRACK_MAX_DELTA_I = 5.0    # safety clamp on integral correction per cycle

# Skip cycles after resume from interruption (window, etc.)
SKIP_CYCLES_AFTER_RESUME = 1
LEARNING_PAUSE_RESUME_MIN = 20  # Pause learning after resume (window close, etc.) to allow stabilization. NB: Corrected duplication in original file comments

# Periodic recalculation interval (seconds) for SmartPI
# This ensures the rate-limit progresses even when temperature sensors don't update frequently
SMARTPI_RECALC_INTERVAL_SEC = 60

# --- Hysteresis Mode (during learning phase) ---
HYST_UPPER_C = 0.5  # ON -> OFF threshold (°C above setpoint)
HYST_LOWER_C = 0.3  # OFF -> ON threshold (°C below setpoint)

# Default deadband around setpoint (°C)
DEFAULT_DEADBAND_C = 0.05

# Absolute hysteresis for deadband exit (reduces oscillations at boundary)
# Enter deadband at |e| < deadband_c, exit only when |e| > deadband_c + hysteresis
# Using absolute value (not multiplicative) ensures consistent behavior across
# different deadband configurations and typical sensor noise levels.
DEADBAND_HYSTERESIS = 0.025

# --- Asymmetric Deadband / Near-band (HEAT only) ---
# Intent (thermal "rule of thumb"):
# - Make the "quiet zone" a bit wider when slightly below the setpoint (e>0) so the controller
#   does not wait too long before restarting after a setpoint decrease.
# - Make the zone tighter above the setpoint (e<0) to reduce overshoot/hunting.
# Guardrails: asymmetry is applied only in HEAT; COOL keeps symmetric logic.

# Deadband (°C) and its hysteresis (°C)
DEADBAND_BELOW_C = 0.06
DEADBAND_ABOVE_C = 0.04
DEADBAND_HYST_BELOW_C = 0.02
DEADBAND_HYST_ABOVE_C = 0.02

# Deadband+ (DB+): minimum holding power when slightly below setpoint inside deadband
DEADBAND_PLUS_MIN_U = 0.08   # 8% duty-cycle
DEADBAND_PLUS_MAX_U = 0.20   # hard cap (safety)

# Micro-leak on integral while in deadband (dt-aware). Value is per "cycle".
INTEGRAL_DEADBAND_MICROLEAK = 0.999

# Near-band asymmetry:
# - below setpoint: use configured near_band_deg (self.near_band_deg)
# - above setpoint: scale it down with a factor
NEAR_BAND_ABOVE_FACTOR = 0.40
NEAR_BAND_HYSTERESIS_C = 0.05

# Setpoint filter parameters (Dual-Track: BOOST + Quadratic Landing)
SP_MIN_LANDING_ZONE = 0.05      # °C — minimum landing zone size
SP_MAX_LANDING_ZONE = 2.0       # °C — maximum landing zone size (safety cap)
SP_LANDING_ZONE_FACTOR = 1.6  # safety multiplier on landing zone to start braking earlier
SP_LANDING_ZONE_MIN_P_FRACTION = 0.3  # minimum P_error as fraction of remaining in landing zone
SP_FILTER_DISABLE_THRESHOLD = 0.10  # °C — below this error, filter turns off to restore stiffness
SP_FILTER_ENABLE_THRESHOLD = 0.50   # °C — delta SP or error needed to reactivate filter

# Error filter time constant
ERROR_FILTER_TAU = 25.0 # Minutes (matches alpha ~0.35 at 10min)


# --- Robust learning / gating constants ---
# Window sizes
B_POINTS_MAX = 40        # OFF samples for b (tau)
A_POINTS_MAX = 25        # ON samples for a
RESIDUAL_HIST_MAX = 60   # Residual history for MAD estimation

# Robust gating
RESIDUAL_GATE_K = 4.5   # |r| > k * sigma_r  -> freeze learning

# Intercept coherence checks (dimensionless ratios)
INTERCEPT_SIGMA_FACTOR = 2.0   # |c| <= factor * sigma_r
INTERCEPT_SCALE_FACTOR = 0.30  # |c| <= factor * median(|y|)

# Tau stability check
B_STABILITY_MAD_RATIO_MAX = 0.60   # MAD(b) / median(b)
LEARN_BOOTSTRAP_COUNT = 10      # Number of learn cycles before applying strict residual gating

# --- SmartPI Robust Learning Constants ---
# Median+MAD Strategy Constants
AB_HISTORY_SIZE = 31      # Keep last 31 (ODD) values
AB_MIN_SAMPLES = 11       # Start learning after 11 (ODD) values
AB_MAD_SIGMA_MULT = 3.0   # Outlier rejection threshold (sigma)

AB_MAD_K = 1.4826         # Sigma scaling factor for MAD
AB_VAL_TOLERANCE = 1e-12  # Small epsilon
LEARN_SAMPLE_MAX = 240          # Max samples history (e.g. 4h @ 1min)
LEARN_Q_HIST_MAX = 200          # History for quantization estimation
DT_MIN_OK = 0.5                 # Min dt (minutes) for valid derivative window
DT_MAX_OK = 30.0                # Max dt (minutes) for valid derivative window
DT_DERIVATIVE_MIN_ABS = 0.03    # Min absolute dT (°C) if quantization unknown
LEARN_QUALITY_THRESHOLD = 0.25  # Min QI quality to accept learning
QUANTIZATION_ROUND_TO = 0.001   # Rounding / binning for quantization detection

# --- SmartPI Learning Window Constants ---
# Learning requires at least one full cycle (measured by _cycle_min)
DT_MAX_MIN = 120
MIN_ABS_DT = 0.03      # °C
DELTA_MIN = 0.2        # °C (Matches DELTA_MIN_ON)
U_OFF_MAX = 0.05
U_ON_MIN = 0.20
DELTA_MIN_OFF = 0.5        # °C
DELTA_MIN_ON = 0.2         # °C

# Episode minimum durations for learning
EPISODE_MIN_DURATION_ON_S = 600   # 10 min
EPISODE_MIN_DURATION_OFF_S = 900  # 15 min


# --- SmartPI Near Band Defaults ---
DEFAULT_NEAR_BAND_DEG = 0.40
DEFAULT_KP_NEAR_FACTOR = 0.7
DEFAULT_KI_NEAR_FACTOR = 0.6


# --- Forced Calibration Constants ---
CALIBRATION_TIMEOUT_MIN = 600  # 10 hours timeout

# --- AutoCalibTrigger Enums ---

class AutoCalibState(str, Enum):
    """State machine states for AutoCalibTrigger."""
    IDLE = "idle"
    WAITING_SNAPSHOT = "waiting_snapshot"
    MONITORING = "monitoring"
    TRIGGERED = "triggered"
    POST_CALIB_CHECK = "post_calib_check"


class AutoCalibWaitingReason(str, Enum):
    """Reason for remaining in waiting_snapshot state."""
    NONE = "none"
    DEADTIME_COOL_PENDING = "deadtime_cool_pending"
    FALLBACK_7D_COUNTDOWN = "fallback_7d_countdown"


# --- AutoCalibTrigger Constants ---
AUTOCALIB_SNAPSHOT_PERIOD_H = 120          # Rolling snapshot period: 5 days
AUTOCALIB_DT_COOL_FALLBACK_DAYS = 7       # Days before fallback if cool deadtime never reliable
AUTOCALIB_COOLDOWN_H = 24                  # Minimum hours between calibrations
AUTOCALIB_A_MAD_THRESHOLD = 0.25          # MAD/med threshold for 'a' stagnation
AUTOCALIB_B_MAD_THRESHOLD = 0.30          # MAD/med threshold for 'b' stagnation
AUTOCALIB_TEXT_GRADIENT_C = 5.0           # Min Tin-Text gradient to check deadtime_cool stagnation
AUTOCALIB_MAX_RETRIES = 3                  # Max retries before declaring model degraded
AUTOCALIB_RETRY_DELAY_H = 6               # Hours between retries
AUTOCALIB_EXIT_NEW_OBS_MIN = 1            # Minimum new observations (a/b) for positive exit

# --- Feed-Forward Gate Constants ---
# Soft gate (Step 2) has been removed.
