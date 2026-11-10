import os
import runpy
import sys
from pathlib import Path
import yaml

config = yaml.safe_load(Path(os.environ.get('GRASPGENX_SERVER_CONFIG', '/opt/deploy/config/server.yaml')).read_text())
root = Path(config['checkpoint_root'])
if config['assets_dir'] == 'auto':
    from graspgenx import get_gripper_descriptions_assets
    assets = Path(get_gripper_descriptions_assets()).parent
else:
    assets = Path(config['assets_dir'])
print(f'Gripper assets resolved to {assets}', flush=True)
for path in [root/'gen', root/'dis', assets/'x_grippers'/config['default_gripper']/'config.json']:
    if not path.exists():
        raise SystemExit(f'Missing model/gripper data: {path}; inspect container download logs')
args = ['python', 'client-server/graspgenx_server.py', '--config', str(root),
        '--assets_dir', str(assets), '--default_gripper', config['default_gripper'],
        '--host', config['host'], '--port', str(config['port'])]
if config.get('fast_inference'):
    args.append('--fast-inference')
fraction = os.environ.get('GRASPGENX_CUDA_MEMORY_FRACTION')
if fraction:
    fraction = float(fraction)
    if not 0 < fraction <= 1:
        raise SystemExit('GRASPGENX_CUDA_MEMORY_FRACTION must be in (0,1]')
    import torch
    torch.cuda.set_per_process_memory_fraction(fraction, device=0)
    sys.argv = args[1:]
    runpy.run_path(args[1], run_name='__main__')
else:
    os.execvp(args[0], args)
