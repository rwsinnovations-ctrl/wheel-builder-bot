
from build123d import *
import numpy as np
import os

def generate_model():
    # Parameters injected from Colab
    N = 32
    D_rim = 600
    D_hub_l = 58
    D_hub_r = 58
    W_l = 35
    W_r = 20
    k = 3
    spoke_dia = 2.0

# --- 1. PARAMETERS FOR TREK 850 (Standard Shimano Hub) ---
    N = 32               # Spoke Count (Check your wheel: could be 36)
    k = 3                # Cross Pattern (Standard is 3-cross)
    spoke_dia = 2.0      # Standard 14g spoke thickness (mm)
    
    # 26" MTB Rim (ISO 559)
    D_rim = 540.0        # Effective Rim Diameter (~540mm for 26")
    
    # Standard Shimano Rear Hub Dimensions
    # Left (Non-Drive Side)
    W_l = 36.0           # Center to Left Flange (mm)
    D_hub_l = 45.0       # Left Flange Diameter (mm)
    
    # Right (Drive Side)
    W_r = 21.0           # Center to Right Flange (mm)
    D_hub_r = 45.0       # Right Flange Diameter (mm)

    
    print(f"Generating wheel with N={N}, k={k}...")
    
    R_rim = D_rim / 2.0
    spokes = []
    
    # Generate 4-spoke cluster
    for rim_idx in [0, 1, 2, 3]:
        is_left = (rim_idx % 2 == 0)
        r_hub = (D_hub_l if is_left else D_hub_r) / 2.0
        z_hub = -W_l if is_left else W_r
        
        # Rim Point
        angle_rim = rim_idx * (2 * np.pi / N)
        p_rim = Vector(R_rim * np.cos(angle_rim), R_rim * np.sin(angle_rim), 0)
        
        # Hub Point
        flange_idx = rim_idx // 2 
        n_flange = N // 2
        direction = 1 if (flange_idx % 2 == 0) else -1
        hub_idx_target = flange_idx + (direction * k)
        theta_hub = hub_idx_target * (2 * np.pi / n_flange)
        p_hub = Vector(r_hub * np.cos(theta_hub), r_hub * np.sin(theta_hub), z_hub)
        
        # Sweep Geometry
        path = Line(p_hub, p_rim)
        with BuildPart() as spoke:
            with BuildSketch(Plane(origin=p_hub, z_dir=path.tangent_at(0))):
                Circle(radius=spoke_dia/2)
            sweep(path=path)
        spokes.append(spoke.part)

    # Bounding cones
    r_hub_l = D_hub_l / 2.0
    z_hub_l = -W_l
    z_apex_l = z_hub_l / (1 - r_hub_l / R_rim)
    with BuildPart() as cone_l_part:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, z_apex_l), (R_rim, 0), (0, 0), close=True)
            make_face()
        revolve(axis=Axis.Z)
    cone_l = cone_l_part.part

    r_hub_r = D_hub_r / 2.0
    z_hub_r = W_r
    z_apex_r = z_hub_r / (1 - r_hub_r / R_rim)
    with BuildPart() as cone_r_part:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, z_apex_r), (R_rim, 0), (0, 0), close=True)
            make_face()
        revolve(axis=Axis.Z)
    cone_r = cone_r_part.part

    final_model = Compound(spokes + [cone_l, cone_r])
    export_step(final_model, "spokes_with_cones.step")
    print("STEP file generated successfully.")

if __name__ == "__main__":
    generate_model()
