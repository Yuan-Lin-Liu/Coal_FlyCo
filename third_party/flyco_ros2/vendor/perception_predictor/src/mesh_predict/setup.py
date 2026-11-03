#!/usr/bin/env python3
"""Python distribution metadata; ROS2 installation is managed by ament."""
from pathlib import Path
from setuptools import find_packages, setup

ROOT = Path(__file__).resolve().parent
requirements = [line.strip() for line in (ROOT / "requirements.txt").read_text().splitlines()
                if line.strip() and not line.lstrip().startswith("#") and line.strip() not in {"zip", "ffmpeg"}]
setup(name="mesh_predict", version="0.1.0", packages=find_packages("src"),
      package_dir={"": "src"}, install_requires=requirements, package_data={"pointnet2_ops": ["_ext-src/include/*", "_ext-src/src/*"]})
