# 위험 차량 쌍 상세 정보 토픽
# 어떤 두 차량이 왜 위험하다고 판단되었는지 설명하는 디버그/분석용 메시지

std_msgs/Header header

# 차량 ID 정보
string object_id_a                # 비교 대상 차량 A의 ID
string object_id_b                # 비교 대상 차량 B의 ID

# 접근로 정보
string approach_a                 # 차량 A의 접근로 ID
string approach_b                 # 차량 B의 접근로 ID

# 우선권 판단 결과
string priority_object_id         # 우선권이 있다고 판단된 차량 ID
string non_priority_object_id     # 양보 또는 정지해야 하는 차량 ID

# 위험 구역 정보
string conflict_zone_id           # 위험 판단에 사용된 conflict zone ID

# ETA 정보
float64 eta_a                     # 차량 A의 conflict zone 도달 예상 시간 (초)
float64 eta_b                     # 차량 B의 conflict zone 도달 예상 시간 (초)
float64 eta_gap                   # 두 차량 ETA 차이의 절대값 (초)

# 최종 판단 결과
bool risky                        # 이 차량 쌍이 최종적으로 위험하다고 판단되었는지 여부
