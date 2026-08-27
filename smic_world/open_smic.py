import omni.usd

# Isaac Sim GUI 시작 시 열기:
#   C:\isaacsim\isaac-sim.bat --exec C:/Users/USER/ISSAC_SIM_test/smic_world/open_smic.py
USD_PATH = r"C:/Users/USER/ISSAC_SIM_test/smic_world/smic_world.usd"

omni.usd.get_context().open_stage(USD_PATH)
print("[open_smic] Opened stage: " + USD_PATH)
