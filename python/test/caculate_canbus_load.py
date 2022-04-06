

cluster_max = 250
object_max = 100
bit_per_frame = 111
frame_rate = 25
bit_rate = 500000

cluster_load = (1 + 2 * cluster_max) * bit_per_frame * frame_rate / bit_rate
object_load = (1 + 3 * object_max) * bit_per_frame * frame_rate / bit_rate

print(cluster_load, object_load)
