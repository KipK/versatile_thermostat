# -*- coding: utf-8 -*-
"""Tests for the AutoTpiManager calculation logic."""

import os
import json
from datetime import datetime, timedelta
from unittest.mock import patch

import pytest
from homeassistant.core import HomeAssistant

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager
from custom_components.versatile_thermostat.const import CONF_TPI_COEF_INT, CONF_TPI_COEF_EXT

def generate_synthetic_data(manager: AutoTpiManager, points: int = 60):
    """Populate `manager._data` with a deterministic series based on physical model.

    Model: dT/dt = -alpha * (T_room - T_ext) + beta * Power
    We use alpha = 1e-4, beta = 5e-4
    """
    alpha = 1e-4
    beta = 5e-4
    t_ext = 10.0
    target_temp = 22.0
    power_percent = 50.0
    power_ratio = power_percent / 100.0

    # Start point
    current_temp = 18.0
    cycle_sec = manager._cycle_min * 60

    now = datetime.now()
    # We generate points ending at 'now'
    start_time = now - timedelta(minutes=points * manager._cycle_min)

    data_points = []

    for i in range(points):
        timestamp = start_time + timedelta(minutes=i * manager._cycle_min)

        # Calculate dT for this step
        # dT/dt = -alpha * (T - Text) + beta * P
        # We use a simple Euler integration
        dt = cycle_sec
        delta_temp = (-alpha * (current_temp - t_ext) + beta * power_ratio) * dt
        current_temp += delta_temp

        data_points.append({
            "timestamp": timestamp.isoformat(),
            "room_temp": current_temp,
            "ext_temp": t_ext,
            "power_percent": power_percent,
            "target_temp": target_temp
        })

    manager._data = data_points


async def test_auto_tpi_calculation(hass: HomeAssistant, tmp_path):
    """Verify that `AutoTpiManager.calculate` produces plausible coefficients."""
    
    # Mock hass.config.path to use tmp_path
    # We need to ensure the directory exists because AutoTpiManager does os.makedirs(os.path.dirname(path))
    # The path used is .storage/versatile_thermostat_{unique_id}_auto_tpi.json
    # So we need to handle the relative path correctly.
    
    def mock_path(path):
        return str(tmp_path / path)

    with patch.object(hass.config, "path", side_effect=mock_path):
        # Initialise the manager with a 5‑minute cycle
        manager = AutoTpiManager(hass, "test_thermostat", cycle_min=5)
        manager.start_learning()

        # Generate deterministic data points
        generate_synthetic_data(manager, points=80)

        # Run the calculation
        result = manager.calculate()

        # The result should be a dictionary with the expected keys
        assert isinstance(result, dict)
        assert CONF_TPI_COEF_INT in result
        assert CONF_TPI_COEF_EXT in result

        # Coefficients must be between 0 and 1 (the manager clamps them)
        int_coef = result[CONF_TPI_COEF_INT]
        ext_coef = result[CONF_TPI_COEF_EXT]
        assert 0.0 <= int_coef <= 1.0
        assert 0.0 <= ext_coef <= 1.0

        # The manager also persists the calculated parameters – verify the file
        storage_path = manager._storage_path
        assert os.path.isfile(storage_path)
        with open(storage_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        assert data.get("calculated_params") == result
