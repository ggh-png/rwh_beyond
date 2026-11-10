#!/usr/bin/env python3
"""Official-client inference smoke test, run only inside the model container."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from graspgenx.serving import GraspGenXClient

parser = argparse.ArgumentParser()
parser.add_argument('--port',type=int,default=5566)
parser.add_argument('--num-grasps',type=int,default=20)
parser.add_argument('--top-k',type=int,default=5)
parser.add_argument('--output',default='/data/logs/local-inference-result.npz')
args=parser.parse_args()
source=Path('/opt/GraspGenX/assets/sample_data/object_pc/1740787815_545011.json')
cloud=np.asarray(json.loads(source.read_text())['pc'],dtype=np.float32)
cloud=cloud[np.isfinite(cloud).all(axis=1)]
cloud=cloud[np.linspace(0,len(cloud)-1,min(len(cloud),2048),dtype=int)]
start=time.monotonic()
with GraspGenXClient(host='127.0.0.1',port=args.port,timeout_ms=120000) as client:
    print('health:',client.health(),flush=True)
    metadata=client.server_metadata
    print('metadata:',json.dumps(metadata),flush=True)
    grasps,scores=client.infer(cloud,gripper_name='piper_hand',num_grasps=args.num_grasps,
        grasp_threshold=-1.0,topk_num_grasps=args.top_k)
    assert len(grasps)>0, 'inference returned no candidates'
    assert grasps.shape==(len(scores),4,4)
    assert np.isfinite(grasps).all() and np.isfinite(scores).all()
    assert np.allclose(grasps[:,3],[0,0,0,1],atol=1e-5)
    np.savez(args.output,point_cloud=cloud,grasps=grasps,confidences=scores)
summary=dict(status='PASS',mock=False,gripper='piper_hand',point_count=len(cloud),
    num_grasps=args.num_grasps,returned=len(grasps),score_min=float(scores.min()),
    score_max=float(scores.max()),client_total_seconds=time.monotonic()-start,
    sample=str(source),output=args.output,metadata=metadata)
Path('/data/logs/local-inference-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2),flush=True)
