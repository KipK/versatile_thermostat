import pytest
from typing import Dict, Any

# Global list to store simulation results
_sim_results = []

@pytest.fixture(scope="function")
def store_sim_result():
    """Fixture to store simulation results for the summary report."""
    def _store(result: Dict[str, Any]):
        _sim_results.append(result)
    return _store

def pytest_terminal_summary(terminalreporter, exitstatus, config):
    """Add a section to terminal summary reporting simulation results."""
    if not _sim_results:
        return

    terminalreporter.section("Auto TPI Simulation Results")
    
    # Define headers
    headers = [
        "Room Type", 
        "Real Coef Ext", "Found Coef Ext", "Error Ext %",
        "Real Coef Int", "Found Coef Int", "Error Int %"
    ]
    
    # Format string for rows
    # Room type needs more space, others are floats
    row_fmt = "{:<15} | {:<13} | {:<14} | {:<11} | {:<13} | {:<14} | {:<11}"
    
    # Print header
    terminalreporter.write_line(row_fmt.format(*headers))
    terminalreporter.write_line("-" * 105)
    
    for res in _sim_results:
        room_type = res.get("room_type", "Unknown")
        
        real_ext = res.get("real_coef_ext", 0.0)
        found_ext = res.get("found_coef_ext", 0.0)
        error_ext = res.get("error_ext", 0.0)
        
        real_int = res.get("real_coef_int", 0.0)
        found_int = res.get("found_coef_int", 0.0)
        error_int = res.get("error_int", 0.0)
        
        terminalreporter.write_line(row_fmt.format(
            room_type,
            f"{real_ext:.4f}",
            f"{found_ext:.4f}",
            f"{error_ext:.1f}%",
            f"{real_int:.4f}",
            f"{found_int:.4f}",
            f"{error_int:.1f}%"
        ))