# External robot assets

These are the existing local robot assets already referenced by the JM/KETI
scenes, copied into the repository without changing geometry or physics.
Only USD asset references are re-anchored to relative paths.

- `unitree_ros/robots/h1_description/`: local converted Unitree H1 assets.
  The upstream Unitree BSD 3-Clause `LICENSE` is included in `unitree_ros/`.
- `atlas_mujoco/atlas_clean.usd`: the existing local converted Atlas asset.
  `UPSTREAM_README.md` preserves the available upstream attribution, including
  its Drake model origin. No additional license was found in this local source directory.
- `robotiq_2f85/robotiq_2f85_flat.usd`: the existing flattened 2F-85 asset downloaded
  from the Isaac Sim robot asset collection by this project's `fetch_2f85.py`.
  No standalone license file was present beside this locally cached asset.
- `../../jm_wb7_pinkik/assets/` contains the project's existing adapted H1/2F-85
  assets. They are packaged without kinematic or physics edits.

This snapshot does not grant a new license to third-party models or claim that
all assets share the Unitree license. Applicable original asset terms remain in force.
`OmniPBR.mdl` is not bundled; it is resolved by the installed Isaac Sim runtime.
