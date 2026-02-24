import pytest
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT
from custom_components.versatile_thermostat.smartpi.controller import SmartPIController
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI

def test_adjust_integral_for_bumpless_transfer():
    """Test that adjusting the integral correctly targets a specific u_pi."""
    controller = SmartPIController("test")
    controller.integral = 10.0
    
    # Current u_pi with kp=0.5, ki=0.1, e_p=2.0 -> u_pi = 0.5 * 2.0 + 0.1 * 10.0 = 2.0
    # Let's say new gains are kp=0.8, ki=0.2
    # We want u_pi to remain 2.0 with the new gains.
    # So 2.0 = 0.8 * 2.0 + 0.2 * new_integral
    # 2.0 = 1.6 + 0.2 * new_integral
    # 0.4 = 0.2 * new_integral -> new_integral = 2.0
    
    controller.adjust_integral_for_bumpless_transfer(
        target_u_pi=2.0,
        kp_new=0.8,
        ki_new=0.2,
        error_p=2.0
    )
    
    # Check if integral was adjusted correctly
    assert pytest.approx(controller.integral, 0.01) == 2.0


@pytest.mark.asyncio
async def test_smartpi_bumpless_gain_change(hass):
    """Test that output remains stable when gains change drastically."""
    algo = SmartPI(
        hass=hass,
        cycle_min=10.0,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="test",
        use_setpoint_filter=False,
    )
    algo.sp_mgr.enabled = False
    algo._output_initialized = True
    algo.est.a_meas_hist = [0.01] * 35
    algo.est.b_meas_hist = [0.001] * 35
    algo.est.learn_ok_count = 35
    algo.est.learn_ok_count_b = 20
    algo.est._b_hat_hist = [0.001] * 20
    algo._tau_reliable = True
    
    from custom_components.versatile_thermostat.smartpi.learning import TauReliability
    # Prevent Python 3.13 generator hang by mocking the entire tau_reliability function
    algo.est.tau_reliability = lambda: TauReliability(reliable=True, tau_min=1000.0)
    # Prevent Python 3.13 generator hang inside the learn function
    algo.est.learn = lambda *args, **kwargs: None
    
    import time
    
    # 1. Start with initial gains
    def mock_calc_run1(*args, **kwargs):
        algo.gain_scheduler.kp = 0.5
        algo.gain_scheduler.ki = 0.05
    algo.gain_scheduler.calculate = mock_calc_run1
    
    algo._last_calculate_time = time.monotonic() - 0.1
    
    algo.calculate(
        target_temp=20.0,
        current_temp=19.8,
        ext_current_temp=20.0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Manually build up some integral so it's not 0
    algo.ctl.integral = 5.0
    integral_before = algo.ctl.integral

    # 2. Simulate a drastic change in gains
    algo.gov.on_cycle_start()
    
    def mock_calc_run2(*args, **kwargs):
        algo.gain_scheduler.kp = 2.0
        algo.gain_scheduler.ki = 0.2
    algo.gain_scheduler.calculate = mock_calc_run2
    
    algo._last_calculate_time = time.monotonic() - 0.1 # Very small delta to avoid massive integral buildup
    
    algo.calculate(
        target_temp=20.0,
        current_temp=19.9,
        ext_current_temp=20.0,
        hvac_mode=VThermHvacMode_HEAT
    )
    integral_after = algo.ctl.integral
    
    # The integral should have adjusted to absorb the gain shock
    assert integral_before != integral_after
    

@pytest.mark.asyncio
async def test_smartpi_bumpless_ff_asymmetric(hass):
    """Test asymmetric bumpless transfer for feed-forward."""
    algo = SmartPI(
        hass=hass,
        cycle_min=10.0,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="test",
        use_setpoint_filter=False,
    )
    algo.sp_mgr.enabled = False
    
    # Mock DeadbandManager properties
    class MockDeadbandMgr:
        def __init__(self):
            self.near_band_below_deg = 1.0
            self.near_band_above_deg = 0.5
            self.in_deadband = False
            self.in_near_band = False
            self.near_band_source = "mock"
        def update(self, *args, **kwargs):
            pass
        def reset(self):
            pass
            
    algo.deadband_mgr = MockDeadbandMgr()
    
    # Mocking FF values and conditions
    from custom_components.versatile_thermostat.smartpi.learning import TauReliability
    
    algo.est.a_meas_hist = [0.01] * 35
    algo.est.b_meas_hist = [0.01] * 35
    algo.est._b_hat_hist = [0.01] * 20
    algo.est.a = 0.01
    algo.est.b = 0.01
    algo.est.learn_ok_count = 35
    algo.est.learn_ok_count_a = 20
    algo.est.learn_ok_count_b = 20
    algo._tau_reliable = True
    algo.est.tau_reliability = lambda: TauReliability(reliable=True, tau_min=100.0)
    algo.est.learn = lambda *args, **kwargs: None
    algo._cycles_since_reset = 50
    algo.ff_warmup_cycles = 1
    algo._output_initialized = True
    
    import time
    algo._last_calc_time = time.time() - 60.0
    
    # 1. Normal state, strong FF
    algo.calculate(
        target_temp=20.0,
        current_temp=19.0, # error = 1.0
        ext_current_temp=19.5, # FF = 1.0 * (20 - 19.5) = 0.5
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.u_ff > 0.1
    integral_base = algo.ctl.integral
    
    # 2. Temperature shoots up, gate hard cuts FF
    algo.gov.on_cycle_start()
    algo.calculate(
        target_temp=20.0,
        current_temp=21.0, # error = -1.0 (exceeds near_band_above_deg of 0.5)
        ext_current_temp=19.5,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.u_ff == 0.0 # Gated
    integral_after_cut = algo.ctl.integral
    
    # 3. Temperature drops back, FF resumes
    algo.gov.on_cycle_start()
    algo.calculate(
        target_temp=20.0,
        current_temp=19.9, # error = 0.1, gate off
        ext_current_temp=19.5,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.u_ff > 0.1 # FF restored
    integral_after_resume = algo.ctl.integral

    # The FF gate opening from ff_cut_above_setpoint is an artificial event, not a physical
    # change in feedforward force. With small Ki the resulting delta-I = delta_u_ff / Ki would
    # be enormous. The bumpless transfer must therefore be skipped in this case.
    # The integral should remain stable (no large spike downward).
    assert abs(integral_after_resume - integral_after_cut) < 0.5
