# 신호 조회 확인 (2026-09-28)

## 확인한 사실

- 최신 pdj1 66b19e3 기준으로 수정했다. 기존 API 키는 변경하지 않았다.
- 서울시 실시간 문서는 잔여시간 `v2xSignalPhaseTimingCurrentInfo`, 현시
  `v2xSignalPhaseCurrentInfo`와 소문자 `apikey` 인자를 안내한다.
- 기존 키, 교차로 2620으로 두 실시간 주소를 조회했을 때 모두 HTTP 404였다.
  응답 본문은 관리자 문의 안내였으며, 이것만으로 권한 문제인지 서버 라우팅 문제인지
  확정할 수 없다. 키 인자 대소문자를 바꾸어도 404였다.
- 이번 직접 점검에서 기존 잔여시간 주소는 timeout, 기존 현시 주소는 404였다.
  사용자가 제시한 이전 429와 이번 응답을 구분해야 한다.
- 공식 문서 예시의 신호 데이터가 보여도 사용 중인 키의 실시간 조회 성공을 의미하지 않는다.

## 코드에서 수정한 부분

실시간 주소와 인증 인자, 교차로 변경 시 호출 간격 우회, HTTP 오류별 재시도 대기,
통합 조회 오류 누락을 수정했다. 구형 제한 API로 자동 fallback하지 않는다.
빨간불은 현시로 판단하며 잔여시간이 양수라는 이유만으로 녹색으로 판정하지 않는다.
신호가 없거나 오래되면 기존처럼 유효하지 않은 상태를 유지한다.

## 아직 해결되지 않은 부분

실시간 API의 유효한 응답을 받지 못했으므로 Pi의 신호 표시 복구를 확인하지 못했다.
횡단보도 중심과 가까운 교차로 선택은 실제 신호등의 연결 관계를 보장하지 않는다.
이전 방향 추정은 진행 방향을 90도 회전해서 반대쪽 접근 시 다른 필드를 선택했다.
후속 수정은 교차로 중심과 횡단보도 축으로 방향을 추정하므로 같은 횡단보도에서는
접근 방향이 바뀌어도 같은 필드를 선택한다. 명시적 방향 데이터가 있으면 우선한다.
다만 제공자의 방향 정의와 현장 신호를 대조한 것은 아니다. 신호 데이터의 유무나
색상에 따라 후보/방향을 고르지 않는다. 표시만 보고 도로에 진입해서는 안 된다.

## 확인 명령

Pi에서 기존 ROS 환경을 불러오고 bringup을 재시작한 뒤:

```bash
ros2 topic echo /crosswalk/guidance
ros2 topic echo /diagnostics
```

`signal_reason`은 HTTP 오류/오래된 데이터/방향 누락 원인,
`signal_direction_source=intersection_arm_inferred`는 교차로 기하 추정값임을 뜻한다.
`unresolved`면 신호 매칭 불가이며 `signal_mapping_reason`에 원인이 나온다.
`required_entry_s`는 진입 필요시간, `crossing_eta_s`는 횡단 중 예상시간,
`signal_remaining_s`는 데이터 나이를 뺀 남은 신호시간이다.
`time_margin_s`는 진입 전에는 신호시간-진입 필요시간, 횡단 중에는 신호시간-ETA다.
횡단 중에는 별도 2초 여유 기준으로 주의를 판정한다. 미확정 값은 JSON null이다.
기존 state/reason 표시 필드는 유지한다. LCD 펌웨어는 이번에 변경하지 않았다.

별도 네트워크 점검은 bringup을 중지한 상태에서 한 번만 실행한다:

```bash
python3 tools/probe_signal_current.py --key-file raspberry_pi/api_key.txt --intersection 2620
```

진단 도구를 반복 실행해 호출 제한을 우회하지 않는다. 키는 출력하지 않는다.

공식 문서:
- https://t-data.seoul.go.kr/dataprovide/trafficdataviewopenapi.do?data_id=10379
- https://t-data.seoul.go.kr/dataprovide/trafficdataviewopenapi.do?data_id=10380
