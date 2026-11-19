# demo2: bir İHA bir görev noktasını tamamladığında (/demo2/task_done) bir kez yayınlanır.
std_msgs/Header header
int32 uav_id
int32 task_id
geometry_msgs/Point position # tamamlandığı andaki İHA konumu (ENU, m)
