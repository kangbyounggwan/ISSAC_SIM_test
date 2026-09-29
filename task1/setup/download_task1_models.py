#!/usr/bin/env python3
"""Download the official inference artifacts used in the Task 1 smoke run."""
import json
from pathlib import Path
import gdown
from huggingface_hub import snapshot_download

root=Path(__file__).resolve().parents[1]
weights=root/'third_party/FoundationPose/weights'
if not all((weights/p/'model_best.pth').is_file() for p in ['2023-10-28-18-33-37','2024-01-11-20-02-45']):
    gdown.download_folder('https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0qzB82i',
                          output=str(weights),quiet=False)
specs=[dict(repo_id='adithyamurali/GraspGenXModel',revision='7c834043c11a11417e31d6d5ea9355801e40a2c1',
            local_dir=str(root/'models/GraspGenXModel')),
       dict(repo_id='adithyamurali/gripper_descriptions',repo_type='dataset',
            revision='19a03c00d19aeaf052d0f6801f0041982d676e8a',local_dir=str(root/'models/gripper_descriptions'),
            allow_patterns=['*.md','LICENSE_ASSETS','pyproject.toml','gripper_descriptions/*.py',
                            'gripper_descriptions/assets/x_grippers/franka_panda/*'])]
for spec in specs:
    snapshot_download(**spec,max_workers=4)
(root/'models/download_manifest.json').write_text(json.dumps(specs,indent=2)+'\n')
