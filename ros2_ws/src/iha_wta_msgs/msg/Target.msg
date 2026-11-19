# Tek bir hedefin konumu (dünya çerçevesi ENU, metre).
int32 id
geometry_msgs/Point position
bool hidden    # hedef bir engelin güvenlik küresi içinde mi (görüş engelli)
bool active    # false ise hedef yakalanmıştır
