
from build123d import *
import numpy as np
import os
from bikewheelcalc import BicycleWheel, Rim, Hub, Spoke
from bikewheelcalc.theory import calc_tor_stiff

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

    wheel = BicycleWheel()
    
    # Create hub
    wheel.hub = Hub(diameter_nds=D_hub_l / 1000., diameter_ds=D_hub_r / 1000.,
                    width_nds=W_l / 1000., width_ds=W_r / 1000.)
    
    # Create rim
    wheel.rim = Rim(radius=D_rim / 2000.,
                    area=82e-6,
                    I_lat=6847e-12,
                    I_rad=1187e-12,
                    J_tor=1e-9,  # placeholder
                    I_warp=0.0,
                    young_mod=70e9,
                    shear_mod=26e9)

    R_rim = D_rim / 2.0

    # Add spokes
    for rim_idx in range(N):
        is_left = (rim_idx % 2 == 0)
        
        # Rim Point
        angle_rim = rim_idx * (2 * np.pi / N)

        # Hub Point
        flange_idx = rim_idx // 2
        n_flange = N // 2
        direction = 1 if (flange_idx % 2 == 0) else -1
        hub_idx_target = flange_idx + (direction * k)
        theta_hub = hub_idx_target * (2 * np.pi / n_flange)

        wheel.spokes.append(Spoke(rim_pt=(wheel.rim.radius, angle_rim, 0.),
                                 hub_pt=(wheel.hub.diameter_nds / 2. if is_left else wheel.hub.diameter_ds / 2.,
                                         theta_hub,
                                         -wheel.hub.width_nds if is_left else wheel.hub.width_ds),
                                 diameter=spoke_dia / 1000.,
                                 young_mod=210e9))

    # Apply spoke tension
    wheel.apply_tension(1000.)

    # Calculate twist stiffness
    tor_stiff = calc_tor_stiff(wheel)
    print(f'Torsional stiffness: {tor_stiff:.2f} [N-m/rad]')

    with open('twist_modulus.txt', 'w') as f:
        f.write(str(tor_stiff))

    # Create 3D model
    spokes = []
    for s in wheel.spokes:
        p_hub = Vector(s.hub_pt[0]*np.cos(s.hub_pt[1]), s.hub_pt[0]*np.sin(s.hub_pt[1]), s.hub_pt[2]) * 1000.
        p_rim = Vector(s.rim_pt[0]*np.cos(s.rim_pt[1]), s.rim_pt[0]*np.sin(s.rim_pt[1]), s.rim_pt[2]) * 1000.

        path = Line(p_hub, p_rim)
        direction = p_rim - p_hub
        with BuildPart() as spoke:
            with BuildSketch(Plane(origin=p_hub, z_dir=direction)):
                Circle(radius=s.diameter*1000./2.)
            sweep(path=path)
        spokes.append(spoke.part)

    final_model = Compound(spokes)
    export_step(final_model, "spokes.step")
    print("STEP file generated successfully.")

if __name__ == "__main__":
    generate_model()
