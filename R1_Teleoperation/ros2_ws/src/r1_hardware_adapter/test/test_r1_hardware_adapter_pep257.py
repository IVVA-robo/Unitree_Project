from pathlib import Path

from ament_pep257.main import main


def test_pep257():
    package_dir = Path(__file__).parents[1]
    assert main([str(package_dir / 'r1_hardware_adapter')]) == 0

