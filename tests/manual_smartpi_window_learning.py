"""Manual SmartPI learning window harness (no external dependencies)."""

from __future__ import annotations

from custom_components.versatile_thermostat import prop_algo_smartpi
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI


class RecordingEstimator(prop_algo_smartpi.ABEstimator):
    """ABEstimator subclass that records learn calls."""

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[float, float, float, float]] = []

    def learn(self, dT_int_per_min: float, u: float, t_int: float, t_ext: float, **kwargs) -> None:
        self.calls.append((dT_int_per_min, u, t_int, t_ext))
        super().learn(dT_int_per_min, u, t_int, t_ext, **kwargs)


def run() -> None:
    fake_time = {"now": 0.0}

    def fake_time_time() -> float:
        return fake_time["now"]

    prop_algo_smartpi.time.time = fake_time_time

    smartpi = SmartPI(
        cycle_min=10.0,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="test",
    )
    smartpi.est = RecordingEstimator()

    ext_temp = 10.0
    temps = [20.02, 20.04, 20.06]
    u_values = [0.30, 0.40, 0.50]

    smartpi.start_new_cycle(u_values[0], 20.0, ext_temp)

    for idx, temp in enumerate(temps):
        fake_time["now"] += 600.0
        smartpi.update_learning(
            current_temp=temp,
            ext_current_temp=ext_temp,
        )
        if idx < 2:
            assert len(smartpi.est.calls) == 0, "learn() should not be called yet"
        if idx + 1 < len(u_values):
            smartpi._cycle_start_state["u_applied"] = u_values[idx + 1]

    assert len(smartpi.est.calls) == 1, "learn() should be called once when window hits threshold"
    print("OK: learning window triggered once after extended window.")


if __name__ == "__main__":
    run()
