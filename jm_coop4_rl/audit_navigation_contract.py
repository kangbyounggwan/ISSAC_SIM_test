"""Read-only checkpoint/interface audit; no simulator, robot commands or training.

The synthetic CPU forward pass checks tensor shape and finiteness only. It is
not evidence of navigation success or that the latest checkpoint is better.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--isaac-root", type=Path, default=Path("C:/isaacsim"))
    args = parser.parse_args()
    if args.out.exists():
        raise RuntimeError("Use a new audit output directory")
    sys.path.append(str(args.isaac_root / "exts/omni.isaac.ml_archive/pip_prebundle"))
    import torch

    constants = {}
    for node in ast.parse((ROOT / "jm_coop4_rl/h1_rack_env.py").read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in {"WALKER_SHA256", "BODY_19", "WALK_DEFAULT_19"}:
                constants[name] = ast.literal_eval(node.value)
    walker_path = ROOT / "artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt"
    nav_dir = ROOT / "jm_coop4_rl/runs/h1_nav_001"
    baseline = ROOT / "jm_h1_ring_pickplace/runs/jm_nav_ring_003_physical_gripper/result.json"
    paths = [walker_path, nav_dir / "model_200.pt", nav_dir / "model_999.pt"]
    before = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    if sha(walker_path) != constants["WALKER_SHA256"]:
        raise RuntimeError("Frozen walker hash differs from the runtime contract")
    walker = torch.jit.load(str(walker_path), map_location="cpu").eval()
    observation = torch.zeros(2, 69)
    observation[:, 8] = -1.0
    observation[1, 9] = 0.1
    with torch.inference_mode():
        action = walker(observation)
    assert list(action.shape) == [2, 19] and torch.isfinite(action).all()
    nav_checks = []
    for name in ("model_200.pt", "model_999.pt"):
        payload = torch.load(nav_dir / name, map_location="cpu", weights_only=True)
        actor = torch.nn.Sequential(
            torch.nn.Linear(18, 128), torch.nn.ELU(),
            torch.nn.Linear(128, 128), torch.nn.ELU(),
            torch.nn.Linear(128, 128), torch.nn.ELU(), torch.nn.Linear(128, 3))
        actor.load_state_dict({k.removeprefix("actor."): v for k, v in payload["model_state_dict"].items()
                               if k.startswith("actor.")}, strict=True)
        actor.eval()
        obs = torch.zeros(2, 18)
        obs[:, 8] = -1.0
        obs[:, 12] = 1.0
        obs[0, 9] = obs[0, 17] = 1.2
        obs[1, 16] = 1.0
        with torch.inference_mode():
            out = torch.tanh(actor(obs))
        assert list(out.shape) == [2, 3] and torch.isfinite(out).all()
        nav_checks.append({"checkpoint": name, "input": 18, "output": 3,
                           "state_dict_strict_load": True, "finite_cpu_forward": True,
                           "task_success_evaluated_by_this_audit": False})
    site = ROOT / "artifacts/upstream/unilm_nav_site_20260929"
    html = (site / "index.html").read_text(encoding="utf-8")
    code_button_disabled = 'href="#" aria-disabled="true"' in html
    site_files = subprocess.check_output(["git", "-C", str(site), "ls-files"], text=True).splitlines()
    version = subprocess.check_output(["git", "-C", str(site), "rev-parse", "HEAD"], text=True).strip()
    after = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    assert before == after
    report = {
        "scope": "read-only checkpoint and source-availability audit",
        "date": "2026-09-29", "pass": True,
        "runtime_torch": torch.__version__, "model_hashes": before,
        "models_unchanged": before == after,
        "walker": {"input": 69, "output": 19, "finite_cpu_forward": True,
                   "controlled_joint_order": constants["BODY_19"], "frozen": True},
        "high_level_navigation": nav_checks,
        "training_record": read_json(nav_dir / "status.json"),
        "deployed_navigation_evidence": read_json(baseline),
        "upstream": {"repository": "https://github.com/UniLM-Nav/UniLM-Nav.github.io",
                     "commit": version, "code_button_disabled": code_button_disabled,
                     "tracked_file_count": len(site_files),
                     "python_files": [f for f in site_files if f.endswith(".py")],
                     "model_files": [f for f in site_files if Path(f).suffix in {".pt", ".pth", ".safetensors", ".onnx"}],
                     "execution_code_integrated": False},
        "simulation_executed": False, "robot_command_sent": False,
        "reward_target_or_limits_changed": False,
    }
    args.out.mkdir(parents=True)
    (args.out / "navigation_contract_audit.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
