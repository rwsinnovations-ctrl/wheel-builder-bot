from build123d import *
from bikewheelcalc import BicycleWheel, Rim, Hub, Spoke
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
    spoke_tension = 1200.0

    print(f"Generating wheel with N={N}, k={k}...")

    # --- Refactor to use bike-wheel-calc ---

    # 1. Create BicycleWheel()
    wheel = BicycleWheel()

    # 2. Define wheel.hub and wheel.rim with properties
    #    Use Rim.general() and calculate box-section properties manually
    #    to avoid the bug in Rim.box()
    w_rim = 21./1000.
    h_rim = 25./1000.
    t_rim = 1.5/1000.
    rim_area = 2*(w_rim+t_rim/2)*t_rim + 2*(h_rim-t_rim/2)*t_rim
    rim_I_rad = 2*(t_rim*(h_rim+t_rim)**3)/12 + 2*((w_rim-t_rim)*t_rim**3/12 + (w_rim-t_rim)*t_rim*(h_rim/2)**2)
    rim_I_lat = 2*(t_rim*(w_rim+t_rim)**3)/12 + 2*((h_rim-t_rim)*t_rim**3/12 + (h_rim-t_rim)*t_rim*(w_rim/2)**2)
    rim_J_tor = 2*t_rim*(w_rim*h_rim)**2 / (w_rim + h_rim)

    wheel.rim = Rim.general(radius=D_rim/2.0 / 1000.,
                            area=rim_area,
                            I_rad=rim_I_rad, I_lat=rim_I_lat, J_tor=rim_J_tor, I_warp=0.0,
                            young_mod=70e9, shear_mod=26e9)

    wheel.hub = Hub(diameter_ds=D_hub_r / 1000.,
                    diameter_nds=D_hub_l / 1000.,
                    width_ds=W_r / 1000.,
                    width_nds=W_l / 1000.)

    # 3. Create and append all Spoke objects to wheel.spokes
    #    Use the lace_cross method and provide young's modulus for steel spokes
    wheel.lace_cross(n_spokes=N,
                     n_cross=k,
                     diameter=spoke_dia / 1000.,
                     young_mod=200e9) # Young's modulus for steel

    # 4. Call wheel.apply_tension()
    wheel.apply_tension(T_avg=spoke_tension)

    print(f"Successfully built wheel model with {len(wheel.spokes)} spokes.")

    # --- Generate 3D geometry from the wheel model ---

    spokes_3d = []
    for s in wheel.spokes:
        # Extract spoke endpoints and convert from meters to millimeters
        p_rim = Vector(s.rim_pt[0] * np.cos(s.rim_pt[1]),
                       s.rim_pt[0] * np.sin(s.rim_pt[1]),
                       s.rim_pt[2]) * 1000.
        p_hub = Vector(s.hub_pt[0] * np.cos(s.hub_pt[1]),
                       s.hub_pt[0] * np.sin(s.hub_pt[1]),
                       s.hub_pt[2]) * 1000.

        # Create 3D spoke geometry
        path = Line(p_hub, p_rim)
        with BuildPart() as spoke_part:
            with BuildSketch(Plane(origin=p_hub, z_dir=path.tangent_at(0))):
                Circle(radius=spoke_dia/2)
            sweep(path=path)
        spokes_3d.append(spoke_part.part)

    # --- Bounding cones ---
    R_rim = D_rim / 2.0

    r_hub_l = wheel.hub.diameter_nds * 1000. / 2.0
    z_hub_l = -wheel.hub.width_nds * 1000.
    z_apex_l = z_hub_l / (1 - r_hub_l / R_rim)
    with BuildPart() as cone_l_part:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, z_apex_l), (R_rim, 0), (0, 0), close=True)
            make_face()
        revolve(axis=Axis.Z)
    cone_l = cone_l_part.part

    r_hub_r = wheel.hub.diameter_ds * 1000. / 2.0
    z_hub_r = wheel.hub.width_ds * 1000.
    z_apex_r = z_hub_r / (1 - r_hub_r / R_rim)
    with BuildPart() as cone_r_part:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline((0, z_apex_r), (R_rim, 0), (0, 0), close=True)
            make_face()
        revolve(axis=Axis.Z)
    cone_r = cone_r_part.part

    final_model = Compound(spokes_3d + [cone_l, cone_r])
    export_step(final_model, "spokes_with_cones.step")
    print("STEP file with cones generated successfully.")

if __name__ == "__main__":
    generate_model()
