#!/usr/bin/env python3
"""Run once in the separate export environment, not ROS Python 2."""
import argparse
import os
import shutil
from pathlib import Path
from ultralytics import YOLOE

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('checkpoint', help='Existing yoloe-26n-seg.pt; cache mobileclip2_b.ts beside it')
args = parser.parse_args()
checkpoint = Path(args.checkpoint).resolve()
if not checkpoint.is_file() or not checkpoint.with_name('mobileclip2_b.ts').is_file():
    parser.error('Cache the original checkpoint and text encoder before export')
destination = Path(__file__).resolve().with_name('yoloe-26n-seg.onnx')
os.chdir(checkpoint.parent)
prompts = ['box', 'cube shaped object']
model = YOLOE(str(checkpoint))
model.set_classes(prompts, model.get_text_pe(prompts))
exported = Path(model.export(format='onnx', opset=17, imgsz=640, dynamic=False,
                             simplify=False, device='cpu', end2end=False)).resolve()
if exported != destination:
    shutil.copyfile(exported, destination)
print('Exported fixed prompts %r to %s' % (prompts, destination))
