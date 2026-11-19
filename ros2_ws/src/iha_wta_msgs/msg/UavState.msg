# Her İHA'nın diğerlerine yayınladığı durum (dünya çerçevesi ENU).
std_msgs/Header header
int32 uav_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
int32 target_id          # -1: hedef yok
string mode              # DIRECT-CHASE, INTERCEPT, LOITER, EMERGENCY-AVOID-..., TAKEOFF ...
bool airborne            # kalkış irtifasına ulaştı mı
