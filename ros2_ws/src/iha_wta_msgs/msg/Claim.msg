# Dağıtık WTA claim mesajı: "bu hedefi ben servis ediyorum".
std_msgs/Header header
int32 uav_id
int32 target_id          # -1: uygun hedef yok
float64 cost             # İHA'nın kendi hesapladığı base_cost
bool continuing          # önceki turda da aynı hedef bu İHA'daydı
string status            # SERVICING-NEW, SERVICING-CONTINUED, NO-AVAILABLE-TARGET
