import omni.usd

# Opened on Isaac Sim GUI startup via: isaac-sim.bat --exec open_world.py
USD_PATH = r"C:/Users/USER/ISSAC_SIM_test/jm_factory_world_atlas_h1.usd"

omni.usd.get_context().open_stage(USD_PATH)
print("[open_world] Opened stage: " + USD_PATH)
