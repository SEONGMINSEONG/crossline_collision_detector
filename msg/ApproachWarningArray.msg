# 접근로별 경고 상태 배열 토픽
# 현재 프레임에서 각 접근로에 내려야 할 경고 상태 목록

std_msgs/Header header

# 접근로별 경고 상태 목록
crossline_collision_detector/ApproachWarning[] warnings   # 접근로별 경고 상태 배열
