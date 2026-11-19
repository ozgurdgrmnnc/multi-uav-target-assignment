# demo2: bir İHA'nın CBBA ajanının komşularına yayınladığı durum.
# Tüm İHA'lar aynı topic'e (/demo2/cbba_bids) yazar ve oradan okur.
# Dizinler görev numarasına (G0, G1, ...) ya da ajan numarasına göredir.
std_msgs/Header header
int32 uav_id                 # gönderen İHA (k)
string state                 # durum makinesi: BEKLE, PAZARLIK, UZLASI, KALKIS, GIT, INIS, YERDE, DONUS, BITTI, BOSTA
int32 round                  # kaçıncı CBBA turu
int32 current_task           # şu an gidilen / üzerinde bulunulan görev (-1: yok)
geometry_msgs/Point position # İHA'nın dünya konumu (ENU, m); comm_range filtresi için
float64[] winning_bids       # y : her görev için bilinen en yüksek teklif
int32[] winners              # z : her görevin kazananı (-1: yok)
float64[] stamps             # s : her ajandan gelen en taze bilginin zamanı (sn)
int32[] bundle               # b : kazanılan görevler, eklenme sırasıyla
int32[] path                 # p : kazanılan görevler, uçuş sırasıyla
bool converged               # bu ajan uzlaşı ilan etti mi
