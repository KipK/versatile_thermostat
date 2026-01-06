# AutoPI Report Script

A standalone Python script to analyze AutoPI algorithm behavior from Home Assistant climate entity history.

## Features

- Connects to Home Assistant REST API
- Extracts AutoPI metrics from climate entity history
- Generates 5 analysis graphs (PNG):
  1. **Learning Progress** - Evolution of a, b, τ parameters
  2. **Temperature Tracking** - Target vs actual temperature with power overlay
  3. **PI Controller State** - Kp, Ki, errors, integral accumulator
  4. **Power Output** - Feed-forward and total power breakdown
  5. **Learning Quality** - Cumulative learning count and reason distribution
- Produces a text summary report

## Installation

```bash
cd dev_tests/autoPI_report
pip install -r requirements.txt
```

## Usage

```bash
python autopi_report.py \
    --url http://homeassistant.local:8123 \
    --token YOUR_LONG_LIVED_ACCESS_TOKEN \
    --entity climate.thermostat_chambre_verte \
    --days 7 \
    --output-dir ./reports \
    --verbose
```

### Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--url` | Yes | - | Home Assistant URL |
| `--token` | Yes | - | Long-Lived Access Token |
| `--entity` | Yes | - | Climate entity ID |
| `--days` | No | 7 | Days of history to fetch |
| `--output-dir` | No | ./reports | Output directory |
| `--verbose` | No | False | Enable debug output |

### Getting a Long-Lived Access Token

1. Go to your Home Assistant profile (click your name, bottom left)
2. Scroll down to "Long-Lived Access Tokens"
3. Click "Create Token"
4. Copy the token (it won't be shown again!)

## Output Files

All files are saved to the `--output-dir` directory:

```
reports/
├── 01_learning_progress.png
├── 02_temperature_tracking.png
├── 03_pi_controller.png
├── 04_power_output.png
├── 05_learning_quality.png
└── autopi_report.txt
```

## Requirements

- Python 3.8+
- `requests` for API calls
- `matplotlib` for graph generation
