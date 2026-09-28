# M4.1 YOLO Detection Model

## Model

- **Model**: YOLO11n (nano) from Ultralytics
- **Dataset**: COCO pretrained
- **Input**: 640x640 RGB image
- **Precision**: FP16
- **Batch**: 1

## TensorRT Engine

| | |
|---|---|
| Built with | TensorRT 10.3.0 (JetPack 6.2.1) |
| Target | Jetson AGX Orin (sm_87) |
| Precision | FP16 |
| Input shape | [1, 3, 640, 640] |
| Output shape | [1, 84, 8400] |
| File | `engines/yolo11n_fp16.engine` |
| Size | 8.15 MiB |
| Build time | ~331 seconds |

### Build Command

```bash
cd "$M4_CODE_ROOT"

/usr/src/tensorrt/bin/trtexec \
  --onnx=models/m4/detection/onnx/yolo11n.onnx \
  --fp16 \
  --saveEngine=models/m4/detection/engines/yolo11n_fp16.engine \
  --memPoolSize=workspace:4096 \
  --minShapes=images:1x3x640x640 \
  --optShapes=images:1x3x640x640 \
  --maxShapes=images:1x3x640x640
```

### Benchmark (standalone, trtexec)

```
Throughput: 338.7 qps
Mean latency: 3.32 ms
GPU compute time: 2.95 ms
P50: 3.32 ms
P95: 3.34 ms
P99: 3.34 ms
```

## ONNX Export

The ONNX model was exported from Ultralytics YOLO11n:

```bash
# If you need to re-export (requires Ultralytics, NOT for runtime)
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
model.export(format='onnx', imgsz=640, opset=12)
```

Note: The runtime does NOT require Ultralytics. Only the exported ONNX is needed.

## Labels

COCO 80-class labels are in `labels/coco.names`.
