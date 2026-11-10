"""Single-worker SAM 3 REP service. No model fallback; every health reply identifies backend."""
import argparse
import os
from pathlib import Path
import time
import numpy as np
import msgpack
import msgpack_numpy
import zmq
from wire import decode_numpy
from segmentation_core import validate_request, select_instance

class Sam3:
    def __init__(self):
        import torch
        from sam3.model_builder import build_sam3_image_model
        from sam3.model.sam3_image_processor import Sam3Processor
        path = os.environ.get('SAM3_CHECKPOINT', '/data/models/sam3.pt')
        if not Path(path).is_file():
            raise FileNotFoundError('SAM3_CHECKPOINT missing; download approved facebook/sam3 weights first')
        torch.set_num_threads(2)
        torch.cuda.set_per_process_memory_fraction(float(os.getenv('SAM3_MEMORY_FRACTION','0.65')))
        self.torch = torch
        self.model = build_sam3_image_model(checkpoint_path=path, load_from_HF=False, enable_inst_interactivity=False, compile=False)
        self.processor = Sam3Processor(self.model, confidence_threshold=0.5)
    def predict(self, image, text, box):
        from PIL import Image
        with self.torch.inference_mode(), self.torch.autocast('cuda', dtype=self.torch.bfloat16):
            state = self.processor.set_image(Image.fromarray(image))
            if text.strip():
                state = self.processor.set_text_prompt(prompt=text, state=state)
            if box.size:
                h,w = image.shape[:2]
                x0,y0,x1,y1 = box
                state = self.processor.add_geometric_prompt(
                    box=[float((x0+x1)/2/w),float((y0+y1)/2/h),float((x1-x0)/w),float((y1-y0)/h)], label=True, state=state)
            return state['masks'][:,0].cpu().numpy(), state['scores'].float().cpu().numpy()

def serve(endpoint, backend, mock=False):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.MAXMSGSIZE, 32*1024*1024)
    socket.bind(endpoint)
    print(f'Segmentation ready: endpoint={endpoint} backend={"mock" if mock else "sam3"}', flush=True)
    try:
        while True:
            try:
                request = msgpack.unpackb(socket.recv(), raw=False, object_hook=decode_numpy)
                if not isinstance(request, dict):
                    raise ValueError('request must be a map')
                action = request.get('action')
                if action == 'health':
                    result = dict(status='ok', backend='mock' if mock else 'sam3', mock=mock)
                elif action == 'segment':
                    started = time.monotonic()
                    image,text,box,point,policy = validate_request(request)
                    masks,scores = backend.predict(image,text,box)
                    mask,score,count = select_instance(masks,scores,point,policy)
                    if mask.shape != image.shape[:2]:
                        raise ValueError('model mask shape differs from input image')
                    result = dict(mask=mask,score=score,instance_count=count,backend='mock' if mock else 'sam3',mock=mock,
                                  elapsed=time.monotonic()-started)
                else:
                    raise ValueError('unknown action')
            except Exception as error:
                # Never log requests or authentication data.
                result = dict(error=f'{type(error).__name__}: {error}')
            socket.send(msgpack.packb(result, default=msgpack_numpy.encode, use_bin_type=True))
    finally:
        socket.close();context.term()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5567')
    args = parser.parse_args()
    serve(args.endpoint, Sam3())
