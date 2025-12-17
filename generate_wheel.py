
from build123d import *
import numpy as np
import os
from bikewheelcalc import BicycleWheel, Rim, Hub, Spoke, ModeMatrix

def generate_model():
    # Update these to match your Trek 850 / Shimano Hub
    N = 32
    k = 3
    D_rim = 540.0        # 26" Rim ERD
    D_hub_l = 45.0       # Left Flange Dia
    D_hub_r = 45.0       # Right Flange Dia
    W_l = 36.0           # Left Flange Offset
    W_r = 21.0           # Right Flange Offset
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

    def calc_rot_stiff(wheel):
        'Calculate rotational (wind-up) stiffness.'

        # Create a ModeMatrix model with 24 modes
        mm = ModeMatrix(wheel, N=24)

        # Calculate stiffness matrix
        K = mm.K_rim(tension=True) + mm.K_spk(smeared_spokes=False, tension=True)

        # Create a unit tangential load at theta=0
        F_ext = mm.F_ext(0., np.array([0., 0., 1., 0.]))

        # Solve for the mode coefficients
        dm = np.linalg.solve(K, F_ext)

        return wheel.rim.radius / mm.rim_def_tan(0., dm)[0]

    # Calculate twist stiffness
    tor_stiff = calc_rot_stiff(wheel)
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
