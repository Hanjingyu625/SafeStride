"""Human-readable guidance; never used as a motor command."""


def describe_crosswalk(state, reason, gps_valid, signal_valid):
    crossing = state in ('CROSSING', 'CROSSING_URGENT')
    if not gps_valid:
        return {'state': '주의' if crossing else '안내 없음',
                'reason': 'GPS 위치 확인 불가; 횡단 중 주변 신호 직접 확인' if crossing
                else 'GPS 위치 확인 불가'}
    if state in ('IDLE', 'EXITING'):
        complete = state == 'EXITING' or reason == 'crossing completed'
        return {'state': '안내 없음',
                'reason': '횡단 완료' if complete else '선택된 횡단보도 없음'}
    if not signal_valid:
        return {'state': '신호 없음',
                'reason': '신호정보 수신 불가; 횡단 중 실제 신호 직접 확인' if crossing
                else '신호정보 수신 불가; 실제 신호 직접 확인'}
    if state == 'CROSSING_URGENT':
        explanations = {
            'continue crossing; remaining signal is tight':
                '남은 신호시간이 부족함; 무리하지 말고 신속히 횡단',
            'crossing timeout; continue assistance and alert':
                '횡단 완료를 확인하지 못함; 현재 위치와 실제 신호 확인',
            'continue crossing; ETA unavailable':
                '남은 횡단시간 계산 불가; 실제 신호 확인',
        }
        return {'state': '주의', 'reason': explanations.get(
            reason, '횡단 상태 확인 필요; 현재 위치와 실제 신호 확인')}
    if state == 'ENTRY_ALLOWED':
        return {'state': '건널 수 있음', 'reason': '예상 횡단시간보다 남은 녹색시간이 충분함'}
    if state == 'CROSSING':
        return {'state': '건널 수 있음', 'reason': '횡단 중; 남은 녹색시간 내 완료 예상'}
    if reason == 'wait; pedestrian signal is red':
        detail = '보행 신호가 빨간불'
    elif state == 'APPROACHING':
        detail = '횡단보도 접근 중; 진입 지점에서 신호 확인 필요'
    else:
        detail = '예상 횡단시간에 비해 남은 녹색시간이 부족함'
    return {'state': '기다려', 'reason': detail}
