# 경고 장치 제어 명령 토픽
# 전광판 장치 1대에 대해 내려보내는 제어 명령 1건을 표현

std_msgs/Header header

# 장치 제어 정보
string device_id                  # 제어 대상 전광판 ID (예: board_1, board_2)
string signal_state               # 전광판이 표시할 상태 (SAFE / WARNING / STOP)
bool active                       # 전광판을 실제로 활성화할지 여부
