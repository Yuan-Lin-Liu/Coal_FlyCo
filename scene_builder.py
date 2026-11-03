"""Small, fully local coal warehouse for Isaac Sim 5.1 / Pegasus.

Call build_scene(stage) after SimulationApp has started.  The builder only
authors USD and never starts Kit, opens assets, or configures the flight stack.
All dimensions are metres; +Z is up. One imported STL heap sits at the centre.
"""

from __future__ import annotations

import math
import hashlib
from pathlib import Path
import struct

import numpy as np
import yaml

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade


ROOT = "/World/CoalWarehouse"


def _material(stage, name, color, roughness=0.8, metallic=0.0):
    material = UsdShade.Material.Define(stage, f"{ROOT}/Materials/{name}")
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _finish(geom, material, collision=False):
    prim = geom.GetPrim()
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    return geom


def _box(stage, path, position, size, material, collision=True, rotate_x=0.0):
    geom = UsdGeom.Cube.Define(stage, f"{ROOT}/{path}")
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rotate_x:
        geom.AddRotateXOp().Set(float(rotate_x))
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    return _finish(geom, material, collision)


def _beam(stage, path, start, end, width, material):
    start, end = Gf.Vec3d(*start), Gf.Vec3d(*end)
    delta = end - start
    geom = UsdGeom.Cube.Define(stage, f"{ROOT}/{path}")
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set((start + end) / 2.0)
    rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), delta.GetNormalized())
    geom.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
    geom.AddScaleOp().Set(Gf.Vec3f(width, width, delta.GetLength()))
    return _finish(geom, material, True)


def _disc(stage, path, center, radius, height, material, collision=False):
    geom = UsdGeom.Cylinder.Define(stage, f"{ROOT}/{path}")
    geom.CreateAxisAttr(UsdGeom.Tokens.z)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(height)
    geom.AddTranslateOp().Set(Gf.Vec3d(*center))
    return _finish(geom, material, collision)


def _pile(stage, settings, material):
    """Preserve the supplied binary STL triangles; centre XY and put its base on Z."""
    source = Path(__file__).resolve().parent / settings["source"]
    data = source.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {source}")
    count = struct.unpack_from("<I", data, 80)[0]
    if not count or len(data) != 84 + count * 50:
        raise ValueError(f"Expected a complete binary STL: {source}")
    records = np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]))
    vertices = records["vertices"].reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise ValueError("STL contains non-finite coordinates")
    points, indices = np.unique(vertices, axis=0, return_inverse=True)
    source_min, source_max = points.min(axis=0), points.max(axis=0)
    scale = float(settings["uniform_scale"])
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("coal_pile.uniform_scale must be positive")
    shift = np.array([*(source_min[:2] + source_max[:2]) / 2, source_min[2]])
    position = np.array([*settings["center_xy"], settings["base_z"]], dtype=float)
    points = (points - shift) * scale + position
    bounds = [points.min(axis=0).tolist(), points.max(axis=0).tolist()]
    geom = UsdGeom.Mesh.Define(stage, f"{ROOT}/CoalPiles/Pile_01")
    geom.CreatePointsAttr(points.tolist())
    geom.CreateFaceVertexCountsAttr([3] * count)
    geom.CreateFaceVertexIndicesAttr(indices.tolist())
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    # The source is an open surface, so show both sides without closing or
    # simplifying the user's model. Ground remains the warehouse slab.
    geom.CreateDoubleSidedAttr(True)
    geom.CreateExtentAttr([Gf.Vec3f(*bounds[0]), Gf.Vec3f(*bounds[1])])
    _finish(geom, material, True)
    UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("none")
    geom.GetPrim().SetCustomDataByKey("source_stl", settings["source"])
    return {
        "path": str(geom.GetPath()), "source": settings["source"],
        "source_sha256": hashlib.sha256(data).hexdigest(),
        "source_dimensions": (source_max - source_min).tolist(),
        "uniform_scale": scale, "bounds": bounds,
        "center": ((points.min(axis=0) + points.max(axis=0)) / 2).tolist(),
        "dimensions": np.ptp(points, axis=0).tolist(),
        "height": float(np.ptp(points[:, 2])),
        "vertices": len(points), "triangles": count,
        "collision": "static triangle mesh",
    }


def _arrow(stage, name, x, y, material):
    geom = UsdGeom.Mesh.Define(stage, f"{ROOT}/Markings/{name}")
    # Flat +X arrow. It is paint, so it must not be a physical obstacle.
    geom.CreatePointsAttr([
        (-0.65, -0.10, 0.007), (0.25, -0.10, 0.007),
        (0.25, -0.32, 0.007), (0.8, 0, 0.007),
        (0.25, 0.32, 0.007), (0.25, 0.10, 0.007), (-0.65, 0.10, 0.007),
    ])
    geom.CreateFaceVertexCountsAttr([3, 3, 3])
    geom.CreateFaceVertexIndicesAttr([0, 1, 5, 0, 5, 6, 2, 3, 4])
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.AddTranslateOp().Set(Gf.Vec3d(x, y, 0))
    _finish(geom, material)


def build_scene(stage, include_roof=True, scene_config=None):
    """Build/replace /World/CoalWarehouse and return JSON-compatible metadata.

    Does not remove /World, create a PhysicsScene, add a drone, set time steps,
    save the stage or start the simulator. Caller owns these integration steps.
    include_roof=False omits only roof sheets; the steel structure remains.
    """
    config = scene_config or yaml.safe_load(
        (Path(__file__).resolve().parent / "config/coal_warehouse.yaml").read_text())
    spawn = config["spawn"]
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    if not stage.GetDefaultPrim():
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    if stage.GetPrimAtPath(ROOT):
        stage.RemovePrim(ROOT)
    root = UsdGeom.Xform.Define(stage, ROOT)
    root.GetPrim().SetCustomDataByKey("description", "Coal warehouse with one imported STL pile; metres; Z-up")
    for group in ("Materials", "Structure", "Roof", "CoalPiles", "Markings", "TakeoffPad", "Lights"):
        UsdGeom.Xform.Define(stage, f"{ROOT}/{group}")

    concrete = _material(stage, "Concrete", (0.32, 0.35, 0.38), 0.96)
    aisle = _material(stage, "ClearAisle", (0.43, 0.46, 0.48), 0.94)
    steel = _material(stage, "BlueSteel", (0.075, 0.19, 0.25), 0.5, 0.35)
    roof = _material(stage, "RoofSheet", (0.27, 0.38, 0.42), 0.67, 0.25)
    orange = _material(stage, "SafetyOrange", (0.95, 0.35, 0.055), 0.7)
    yellow = _material(stage, "LaneYellow", (0.98, 0.69, 0.12), 0.75)
    white = _material(stage, "PadWhite", (0.9, 0.93, 0.93), 0.8)
    pad = _material(stage, "PadBlue", (0.055, 0.29, 0.39), 0.92)
    coal = [
        _material(stage, f"Coal{i}", color, roughness=0.96 - i * 0.015)
        for i, color in enumerate(((0.034, 0.041, 0.047), (0.044, 0.051, 0.058),
                                   (0.052, 0.060, 0.068), (0.064, 0.071, 0.078)))
    ]

    # The base slab is the sole collision ground; painted patches avoid seams.
    _box(stage, "Ground", (0, 0, -0.18), (64, 44, 0.36), concrete)
    # Two clear side lanes replace the old aisle through the pile footprint.
    for side in (-1, 1):
        suffix = "S" if side < 0 else "N"
        _box(stage, f"Markings/Aisle_{suffix}", (0, side * 8.5, 0.002),
             (44, 5.0, 0.004), aisle, False)
        for edge in (6.0, 11.0):
            _box(stage, f"Markings/LaneEdge_{suffix}_{int(edge)}", (0, side * edge, 0.008),
                 (44, 0.10, 0.012), yellow, False)
        _box(stage, f"Markings/OuterEdge_{suffix}", (0, side * 15.8, 0.008),
             (49, 0.10, 0.012), yellow, False)
        for i, x in enumerate((-18, -10, 0, 10, 18)):
            _arrow(stage, f"Direction_{suffix}_{i:02d}", x, side * 8.5, yellow)

    # Open sides and two full-width end openings; posts are outside flight lanes.
    for bay, x in enumerate((-24, -12, 0, 12, 24)):
        for side in (-1, 1):
            suffix = f"{bay}_{'S' if side < 0 else 'N'}"
            y = 15 * side
            _box(stage, f"Structure/Foot_{suffix}", (x, y, 0.2), (0.78, 0.78, 0.4), concrete)
            _box(stage, f"Structure/Post_{suffix}", (x, y, 4.1), (0.22, 0.28, 7.8), steel)
            _box(stage, f"Structure/Guard_{suffix}", (x, y, 0.85), (0.31, 0.37, 0.9), orange)
            _beam(stage, f"Structure/Rafter_{suffix}", (x, y, 8), (x, 0, 9.2), 0.18, steel)
            _beam(stage, f"Structure/Brace_{suffix}", (x, y, 6.1), (x, y - side * 2.2, 8.18), 0.12, steel)
        _beam(stage, f"Structure/Tie_{bay}", (x, -15, 7.9), (x, 15, 7.9), 0.12, steel)
        # Short vertical king post and web diagonals retain a recognisable truss.
        _beam(stage, f"Structure/KingPost_{bay}", (x, 0, 7.9), (x, 0, 9.2), 0.1, steel)
        for side in (-1, 1):
            _beam(stage, f"Structure/Web_{bay}_{'S' if side < 0 else 'N'}",
                  (x, side * 7.5, 7.9), (x, 0, 9.2), 0.09, steel)
    for side in (-1, 1):
        suffix = "S" if side < 0 else "N"
        for line, y in enumerate((15 * side, 8 * side)):
            z = 9.2 - abs(y) * 0.08
            _beam(stage, f"Structure/Purlin_{suffix}_{line}", (-24, y, z), (24, y, z), 0.16, steel)
        if include_roof:
            # Two seven-metre strips leave a broad central roof opening.
            _box(stage, f"Roof/Sheet_{suffix}", (0, side * 11.5, 8.32),
                 (48.6, 7.2, 0.055), roof, True, -side * math.degrees(math.atan(0.08)))

    piles = [_pile(stage, config["coal_pile"], coal[0])]

    # Paint shares the physical slab and marks the configured spawn position.
    px, py = spawn[:2]
    _disc(stage, "TakeoffPad/OuterRing", (px, py, 0.010), 1.8, 0.006, white)
    _disc(stage, "TakeoffPad/BlueCircle", (px, py, 0.014), 1.64, 0.006, pad)
    for name, position, size in (
        ("HLeft", (px - .45, py, 0.020), (0.16, 1.35, 0.006)),
        ("HRight", (px + .45, py, 0.020), (0.16, 1.35, 0.006)),
        ("HCross", (px, py, 0.020), (0.9, 0.16, 0.006)),
    ):
        _box(stage, f"TakeoffPad/{name}", position, size, white, False)

    sun = UsdLux.DistantLight.Define(stage, f"{ROOT}/Lights/Sun")
    sun.CreateIntensityAttr(1500)
    sun.CreateAngleAttr(1.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-38, -25, -30))
    sky = UsdLux.DomeLight.Define(stage, f"{ROOT}/Lights/SkyFill")
    sky.CreateIntensityAttr(300)
    sky.CreateColorAttr(Gf.Vec3f(0.70, 0.81, 1.0))

    return {
        "root_path": ROOT,
        "units": "metres", "up_axis": "Z",
        "warehouse_dimensions": [48.0, 30.0, 9.2],
        "slab_bounds": [[-32, -22, -0.36], [32, 22, 0]],
        "start_position": list(spawn),
        "start_yaw_degrees": config.get("spawn_yaw_degrees", 0.0),
        "clear_side_aisles": [
            {"x": [-22, 22], "y": [-11, -6], "height": 7.7},
            {"x": [-22, 22], "y": [6, 11], "height": 7.7}],
        "suggested_flight_bounds": [[-25, -13.5, 0.35], [25, 13.5, 6.5]],
        "takeoff_pad_center": [spawn[0], spawn[1], 0.0],
        "pile_count": len(piles), "piles": piles,
        "pile_triangles": sum(pile["triangles"] for pile in piles),
        "roof_path": f"{ROOT}/Roof", "roof_included": bool(include_roof),
        "source_assets": [config["coal_pile"]["source"]],
        "recommended_camera_eye": config["overview_eye"],
        "recommended_camera_target": config["overview_target"],
    }
