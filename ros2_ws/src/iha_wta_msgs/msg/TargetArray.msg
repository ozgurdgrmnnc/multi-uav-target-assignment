std_msgs/Header header
uint32 step              # ölçüm adımı (world_node her step_period saniyede bir artırır)
float64 sim_step         # sürekli senaryo zamanı (adım birimi)
bool started             # görev başladı mı (tüm İHA'lar havada)
bool mission_complete    # tüm hedefler yakalandı mı
Target[] targets
