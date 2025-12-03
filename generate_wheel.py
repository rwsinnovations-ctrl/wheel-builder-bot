
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
            with BuildSketch(Plane(origin=p_hub, z_dir=path.direction_at(0))):
                Circle(radius=spoke_dia/2)
            sweep(path=path)
        spokes.append(spoke.part)

    final_model = Compound(spokes)
    export_step(final_model, "spokes.step")
    print("STEP file generated successfully.")

if __name__ == "__main__":
    generate_model()
