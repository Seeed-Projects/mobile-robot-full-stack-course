# System-level status beacon emitted by bev_system_monitor.
uint8 INITIALIZING = 0
uint8 READY        = 1
uint8 DEGRADED     = 2
uint8 ERROR        = 3
uint8 SAFE_STOP    = 4

uint8 state
string message

float32 camera_fps
float32 camera_sync_delta_ms
float32 inference_fps
float32 inference_latency_ms
float32 total_latency_ms
uint32 detected_objects
# aggregated system state inputs (0.0 = unavailable)
float32 gpu_usage
float32 gpu_memory