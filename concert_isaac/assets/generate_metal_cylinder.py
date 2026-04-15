#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

def build_usda(outer_radius: float, wall_thickness: float, height: float, segments: int,
               base_color=(0.72, 0.74, 0.78), metallic=1.0, roughness=0.12, ior=1.5) -> str:
    inner_radius = outer_radius - wall_thickness
    if inner_radius <= 0:
        raise ValueError("wall_thickness must be smaller than outer_radius")
    if segments < 8:
        raise ValueError("segments must be >= 8")

    points = []
    face_counts = []
    face_indices = []

    def add_quad(a, b, c, d):
        face_counts.append(4)
        face_indices.extend([a, b, c, d])

    outer_bot, outer_top, inner_bot, inner_top = [], [], [], []
    for i in range(segments):
        ang = 2 * math.pi * i / segments
        c, s = math.cos(ang), math.sin(ang)
        outer_bot.append(len(points)); points.append((outer_radius * c, -height / 2, outer_radius * s))
        outer_top.append(len(points)); points.append((outer_radius * c,  height / 2, outer_radius * s))
        inner_bot.append(len(points)); points.append((inner_radius * c, -height / 2, inner_radius * s))
        inner_top.append(len(points)); points.append((inner_radius * c,  height / 2, inner_radius * s))

    for i in range(segments):
        j = (i + 1) % segments
        add_quad(outer_bot[i], outer_bot[j], outer_top[j], outer_top[i])      # outer wall
        add_quad(inner_bot[i], inner_top[i], inner_top[j], inner_bot[j])      # inner wall
        add_quad(outer_top[i], outer_top[j], inner_top[j], inner_top[i])      # top rim
        add_quad(outer_bot[i], inner_bot[i], inner_bot[j], outer_bot[j])      # bottom rim

    def fmt3(v):
        return f"({v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f})"

    points_str = ",\n            ".join(fmt3(p) for p in points)
    counts_str = ", ".join(str(c) for c in face_counts)
    indices_str = ", ".join(str(i) for i in face_indices)
    base_color_str = f"({base_color[0]:.4f}, {base_color[1]:.4f}, {base_color[2]:.4f})"

    template = r"""#usda 1.0
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
                color3f inputs:diffuseColor = (0.02, 0.02, 0.02)
                color3f inputs:specularColor = (0.9, 0.9, 0.9)
                color3f inputs:emissiveColor = (0, 0, 0)
                float inputs:metallic = {metallic:.4f}
                float inputs:roughness = {roughness:.4f}
                float inputs:ior = {ior:.4f}
                float inputs:clearcoat = 0
                float inputs:clearcoatRoughness = 0.01
                normal3f outputs:surface
            }}
        }}
    }}

    def Mesh "Tube"
    {{
        rel material:binding = </MetalTubeAsset/Looks/MetalMaterial>
        point3f[] points = [
            {points_str}
        ]
        int[] faceVertexCounts = [{counts_str}]
        int[] faceVertexIndices = [{indices_str}]
        uniform token subdivisionScheme = "none"
        bool doubleSided = 0
        color3f[] primvars:displayColor = [{base_color_str}]
        uniform token primvars:displayColor:interpolation = "constant"
        float3[] extent = [(-{outer_radius:.6f}, -{half_height:.6f}, -{outer_radius:.6f}), ({outer_radius:.6f}, {half_height:.6f}, {outer_radius:.6f})]
    }}
}}
"""
    return template.format(
        metallic=metallic,
        roughness=roughness,
        ior=ior,
        points_str=points_str,
        counts_str=counts_str,
        indices_str=indices_str,
        base_color_str=base_color_str,
        outer_radius=outer_radius,
        half_height=height/2,
    )

def main():
    parser = argparse.ArgumentParser(description="Generate a configurable metal tube USDA asset.")
    parser.add_argument("--outer-radius", type=float, default=0.05, help="Outer radius in meters")
    parser.add_argument("--wall-thickness", type=float, default=0.005, help="Wall thickness in meters")
    parser.add_argument("--height", type=float, default=1.0, help="Tube length/height in meters")
    parser.add_argument("--segments", type=int, default=64, help="Radial segment count")
    parser.add_argument("--metallic", type=float, default=1.0, help="PBR metallic value")
    parser.add_argument("--roughness", type=float, default=0.12, help="PBR roughness value")
    parser.add_argument("--ior", type=float, default=1.5, help="Index of refraction")
    parser.add_argument("--base-color", type=float, nargs=3, default=(0.72, 0.74, 0.78), metavar=("R", "G", "B"),
                        help="RGB base color")
    parser.add_argument("-o", "--output", type=Path, default=Path("metal_tube_configurable.usda"),
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