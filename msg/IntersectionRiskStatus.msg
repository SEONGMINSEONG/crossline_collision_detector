# 교차로 전체 위험 상태 토픽
# 현재 교차로에 충돌 위험이 있는지 요약해서 확인

std_msgs/Header header

# 전체 상태
bool has_risk                     # 현재 위험 차량 쌍이 있는지 여부
uint32 tracked_objects_count      # 분석 대상 차량 수
uint32 conflicting_pairs_count    # 위험하다고 판단된 차량 쌍 수

# ETA 기반 위험 판단 정보
float64 min_eta_gap               # 위험 차량 쌍 중 최소 ETA 차이 (초)
float64 eta_gap_threshold         # 위험 판정에 사용한 ETA 차이 임계값 (초)

# 처리 시간 (ms)
float64 processing_time_ms        # 현재 프레임 처리 시간
