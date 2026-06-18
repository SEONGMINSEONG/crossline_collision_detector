# 접근로별 경고 상태 토픽
# 특정 접근로에 SAFE / WARNING / STOP 중 어떤 신호를 줄지 표현

std_msgs/Header header

# 접근로 상태
string approach_id                # 경고 대상 접근로 ID (예: 북쪽, 동쪽, 남쪽, 서쪽)
string signal_state               # 현재 접근로 신호 상태 (SAFE / WARNING / STOP)
string reason                     # 신호 상태 결정 이유 (예: PRIORITY_WARNING, PRIORITY_STOP)

# ETA 정보
float64 eta_gap                   # 상대 차량과의 ETA 차이 (초)
