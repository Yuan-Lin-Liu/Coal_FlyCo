"""Source-tree entrypoint for the simulation's camera visualization node."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2/flyco_real'))
from flyco_real.camera_fov import main
if __name__ == '__main__':
    main()
