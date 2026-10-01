"""Verify the packaged actor without starting or modifying a simulator."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--load', action='store_true', help='Also smoke-test with installed PyTorch on CPU')
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent
    spec = json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    path = directory/spec['file']
    if path.parent != directory:
        raise ValueError('Model must be packaged beside its manifest')
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if len(data) != spec['size_bytes'] or digest != spec['sha256']:
        raise ValueError('WB5 model size/hash mismatch')
    if len(spec['body_joint_order']) != spec['output_dim'] or len(set(spec['body_joint_order'])) != spec['output_dim']:
        raise ValueError('Invalid joint mapping')
    if sum(field['dim'] for field in spec['observation_order']) != spec['input_dim']:
        raise ValueError('Invalid observation layout')
    result = dict(hash_verified=True, sha256=digest, bytes=len(data),
                  full_ppo_resume_checkpoint=False, locomotion_performance_verified=False)
    if args.load:
        import torch
        torch.set_num_threads(1)
        model = torch.jit.load(str(path), map_location='cpu').eval()
        with torch.inference_mode():
            output = model(torch.zeros(2, spec['input_dim']))
        if tuple(output.shape) != (2, spec['output_dim']) or not torch.isfinite(output).all():
            raise ValueError('Model output dimension/finiteness check failed')
        result.update(torchscript_load_passed=True, output_shape=list(output.shape))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
