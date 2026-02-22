"""Test integral hold during deadtime window."""

import time
import pytest
from unittest.mock import MagicMock

from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.smartpi.const import AB_HISTORY_SIZE
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT


def make_smartpi(**kwargs):
    defaults = dict(
        hass=MagicMock(),
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestIntegralHold",
        debug_mode=True,
    )
    defaults.update(kwargs)
    return SmartPI(**defaults)


def _force_stable_phase(pi):
    """Force SmartPI into STABLE phase by populating measurement history."""
    for _ in range(AB_HISTORY_SIZE + 1):
        pi.est.a_meas_hist.append(0.01)
        pi.est.b_meas_hist.append(0.002)
    pi._output_initialized = True


def _set_reliable_deadtime(pi, dt_s: float):
    """Inject a reliable heat deadtime and a fake episode start."""
    pi.dt_est.deadtime_heat_s = dt_s
    pi.dt_est.deadtime_heat_reliable = True
    pi._t_heat_episode_start = time.monotonic()  # episode started just now


class TestIntegralHoldDuringDeadtime:

    def test_in_deadtime_window_returns_true(self):
        """in_deadtime_window is True when inside the deadtime window."""
        pi = make_smartpi()
        _set_reliable_deadtime(pi, dt_s=300.0)  # 5-minute deadtime
        assert pi.in_deadtime_window is True

    def test_outside_deadtime_window_returns_false(self):
        """in_deadtime_window is False when deadtime has expired."""
        pi = make_smartpi()
        pi.dt_est.deadtime_heat_s = 1.0
        pi.dt_est.deadtime_heat_reliable = True
        # Episode started far in the past
        pi._t_heat_episode_start = time.monotonic() - 10.0
        assert pi.in_deadtime_window is False

    def test_no_reliable_deadtime_returns_false(self):
        """in_deadtime_window is False when deadtime is not reliable."""
        pi = make_smartpi()
        pi.dt_est.deadtime_heat_reliable = False
        pi._t_heat_episode_start = time.monotonic()
        assert pi.in_deadtime_window is False

    def test_compute_pwm_receives_hold_true_during_deadtime(self):
        """calculate() must pass integrator_hold=True to compute_pwm when in deadtime window."""
        from unittest.mock import patch as _patch
        pi = make_smartpi()
        _force_stable_phase(pi)
        _set_reliable_deadtime(pi, dt_s=300.0)
        assert pi.in_deadtime_window is True

        with _patch.object(pi.ctl, 'compute_pwm', wraps=pi.ctl.compute_pwm) as mock_pwm:
            pi.calculate(
                target_temp=20.0,
                current_temp=18.0,
                ext_current_temp=5.0,
                hvac_mode=VThermHvacMode_HEAT,
            )

        mock_pwm.assert_called_once()
        # integrator_hold is positional arg index 8 (after self: error, error_p, kp, ki, u_ff, dt_min, cycle_min, in_deadband, integrator_hold)
        integrator_hold_arg = mock_pwm.call_args[0][8]
        assert integrator_hold_arg is True, (
            f"compute_pwm should receive integrator_hold=True during deadtime, got {integrator_hold_arg}"
        )

    def test_deadtime_does_not_set_hold_outside_window(self):
        """Deadtime code path must not set integrator_hold=True when outside deadtime window.

        We spy on gov.determine_regime to capture the integrator_hold value
        BEFORE governance may independently freeze the integrator for other reasons
        (e.g. saturation). Index 2 is the integrator_hold positional arg.
        """
        from unittest.mock import patch as _patch
        pi = make_smartpi()
        _force_stable_phase(pi)

        # No reliable deadtime → in_deadtime_window == False
        pi.dt_est.deadtime_heat_reliable = False
        assert pi.in_deadtime_window is False

        with _patch.object(pi.gov, 'determine_regime', wraps=pi.gov.determine_regime) as mock_gov:
            pi.calculate(
                target_temp=20.0,
                current_temp=18.0,
                ext_current_temp=5.0,
                hvac_mode=VThermHvacMode_HEAT,
            )

        mock_gov.assert_called_once()
        # integrator_hold is positional arg index 2 (phase, ext_temp, integrator_hold, ...)
        hold_at_governance_entry = mock_gov.call_args[0][2]
        assert hold_at_governance_entry is False, (
            f"Deadtime must not set integrator_hold before governance when outside window, got {hold_at_governance_entry}"
        )
