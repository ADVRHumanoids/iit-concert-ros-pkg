#!/usr/bin/env python3
import argparse
import math
from pathlib import Path


def build_usda(outer_radius: float, wall_thickness: float, height: float, segments: int,
               base_color=(0.72, 0.74, 0.78), metallic=0.0, roughness=0.5, ior=1.5) -> str:
    """Build a USDA string for a hollow metal cylinder.

    Visual mesh  : hollow tube (outer wall + inner wall + top/bottom rims).
    Collision    : solid cylinder approximated as convexHull on a separate invisible Mesh
                   (much cheaper than per-triangle collision on the hollow geometry).
    Colour       : base_color is placed in diffuseColor (metallic-PBR workflow:
                   diffuseColor IS the albedo; setting it near-black renders black).
    """
    inner_radius = outer_radius - wall_thickness
    if inner_radius <= 0:
        raise ValueError("wall_thickness must be smaller than outer_radius")
    if segments < 8:
        raise ValueError("segments must be >= 8")

    # ------------------------------------------------------------------ #
    # Visual mesh: hollow tube                                             #
    # ------------------------------------------------------------------ #
    vis_points = []
    vis_counts = []
    vis_indices = []

    def add_quad(pts, counts, indices, a, b, c, d):
        counts.append(4)
        indices.extend([a, b, c, d])

    outer_bot, outer_top, inner_bot, inner_top = [], [], [], []
    for i in range(segments):
        ang = 2 * math.pi * i / segments
        cx, cz = math.cos(ang), math.sin(ang)
        outer_bot.append(len(vis_points)); vis_points.append((outer_radius * cx, -height / 2, outer_radius * cz))
        outer_top.append(len(vis_points)); vis_points.append((outer_radius * cx,  height / 2, outer_radius * cz))
        inner_bot.append(len(vis_points)); vis_points.append((inner_radius * cx, -height / 2, inner_radius * cz))
        inner_top.append(len(vis_points)); vis_points.append((inner_radius * cx,  height / 2, inner_radius * cz))

    for i in range(segments):
        j = (i + 1) % segments
        add_quad(vis_points, vis_counts, vis_indices,
                 outer_bot[i], outer_bot[j], outer_top[j], outer_top[i])   # outer wall
        add_quad(vis_points, vis_counts, vis_indices,
                 inner_bot[i], inner_top[i], inner_top[j], inner_bot[j])   # inner wall
        add_quad(vis_points, vis_counts, vis_indices,
                 outer_top[i], outer_top[j], inner_top[j], inner_top[i])   # top rim
        add_quad(vis_points, vis_counts, vis_indices,
                 outer_bot[i], inner_bot[i], inner_bot[j], outer_bot[j])   # bottom rim

    # ------------------------------------------------------------------ #
    # Collision mesh: solid cylinder (outer ring + cap centres)           #
    # convexHull over this is fast and gives a tight cylinder shape.      #
    # ------------------------------------------------------------------ #
    col_points = []
    col_counts = []
    col_indices = []
    col_bot, col_top = [], []
    for i in range(segments):
        ang = 2 * math.pi * i / segments
        cx, cz = math.cos(ang), math.sin(ang)
        col_bot.append(len(col_points)); col_points.append((outer_radius * cx, -height / 2, outer_radius * cz))
        col_top.append(len(col_points)); col_points.append((outer_radius * cx,  height / 2, outer_radius * cz))

    col_bot_center = len(col_points); col_points.append((0.0, -height / 2, 0.0))
    col_top_center = len(col_points); col_points.append((0.0,  height / 2, 0.0))

    for i in range(segments):
        j = (i + 1) % segments
        add_quad(col_points, col_counts, col_indices,
                 col_bot[i], col_bot[j], col_top[j], col_top[i])
        col_counts.append(3); col_indices.extend([col_bot_center, col_bot[j], col_bot[i]])
        col_counts.append(3); col_indices.extend([col_top_center, col_top[i], col_top[j]])

    # ------------------------------------------------------------------ #
    # Format helpers                                                       #
    # ------------------------------------------------------------------ #
    def fmt3(v):
        return f"({v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f})"

    def fmt_pts(pts):
        return ",\n            ".join(fmt3(p) for p in pts)

    vis_points_str  = fmt_pts(vis_points)
    vis_counts_str  = ", ".join(str(c) for c in vis_counts)
    vis_indices_str = ", ".join(str(i) for i in vis_indices)

    col_points_str  = fmt_pts(col_points)
    col_counts_str  = ", ".join(str(c) for c in col_counts)
    col_indices_str = ", ".join(str(i) for i in col_indices)

    r, g, b = base_color
    base_color_str = f"({r:.4f}, {g:.4f}, {b:.4f})"

    usda = f"""#usda 1.0
(
    defaultPrim = "MetalTubeAsset"
    metersPerUnit = 1
    upAxis = "Y"
)

def Xform "MetalTubeAsset" (
    kind = "component"
    assetInfo = {{
        string name = "MetalTubeConfigurable"
    }}
)
{{
    def Scope "Looks"
    {{
        def Material "MetalMaterial"
        {{
            token outputs:surface.connect = </MetalTubeAsset/Looks/MetalMaterial/PBR.outputs:surface>

            def Shader "PBR"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {base_color_str}
                float inputs:metallic = {metallic:.4f}
                float inputs:roughness = {roughness:.4f}
                float inputs:ior = {ior:.4f}
                token outputs:surface
            }}
        }}
    }}

    def Mesh "Tube"
    {{
        rel material:binding = </MetalTubeAsset/Looks/MetalMaterial>
        point3f[] points = [
            {vis_points_str}
        ]
        int[] faceVertexCounts = [{vis_counts_str}]
        int[] faceVertexIndices = [{vis_indices_str}]
        uniform token subdivisionScheme = "none"
        bool doubleSided = 0
        float3[] extent = [(-{outer_radius:.6f}, -{height/2:.6f}, -{outer_radius:.6f}), ({outer_radius:.6f}, {height/2:.6f}, {outer_radius:.6f})]
    }}

    def Mesh "CollisionCylinder"
    {{
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
        uniform token physics:approximation = "convexHull"
        purpose = "guide"
        visibility = "invisible"
        point3f[] points = [
            {col_points_str}
        ]
        int[] faceVertexCounts = [{col_counts_str}]
        int[] faceVertexIndices = [{col_indices_str}]
        uniform token subdivisionScheme = "none"
        float3[] extent = [(-{outer_radius:.6f}, -{height/2:.6f}, -{outer_radius:.6f}), ({outer_radius:.6f}, {height/2:.6f}, {outer_radius:.6f})]
    }}

    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    bool physics:rigidBodyEnabled = 0
}}
"""
    return usda


def main():
    parser = argparse.ArgumentParser(description="Generate a configurable hollow metal cylinder USDA asset.")
    parser.add_argument("--outer-radius",   type=float, default=0.05,  help="Outer radius in meters")
    parser.add_argument("--wall-thickness", type=float, default=0.005, help="Wall thickness in meters")
    parser.add_argument("--height",         type=float, default=1.0,   help="Cylinder height in meters")
    parser.add_argument("--segments",       type=int,   default=64,    help="Radial segment count")
    parser.add_argument("--metallic",       type=float, default=0.0,   help="PBR metallic value [0-1]")
    parser.add_argument("--roughness",      type=float, default=0.5,   help="PBR roughness value [0-1]")
    parser.add_argument("--ior",            type=float, default=1.5,   help="Index of refraction")
    parser.add_argument("--base-color",     type=float, nargs=3,
                        default=(0.72, 0.74, 0.78), metavar=("R", "G", "B"),
                        help="Linear RGB base colour (default: aluminium grey 0.72 0.74 0.78)")
    parser.add_argument("-o", "--output",   type=Path,
                        default=Path("metal_tube_configurable.usda"),
                        help="Output .usda file path")
    args = parser.parse_args()

    usda = build_usda(
        outer_radius=args.outer_radius,
        wall_thickness=args.wall_thickness,
        height=args.height,
        segments=args.segments,
        base_color=tuple(args.base_color),
        metallic=args.metallic,
        roughness=args.roughness,
        ior=args.ior,
    )
    args.output.write_text(usda, encoding="utf-8")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
