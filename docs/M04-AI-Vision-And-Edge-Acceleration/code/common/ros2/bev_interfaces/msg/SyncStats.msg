# Six-camera synchronization statistics.
std_msgs/Header header
# spread across the 6 header stamps of the latest accepted frame set
float64 max_timestamp_delta_ms
float64 mean_timestamp_delta_ms
uint32 dropped_frames        # frame sets rejected (missing / stale input)
uint32 incomplete_framesets  # rejected because not all 6 arrived on time
uint32 stale_framesets       # rejected because max_delta > allowed
uint32 published_framesets   # frame sets published so far