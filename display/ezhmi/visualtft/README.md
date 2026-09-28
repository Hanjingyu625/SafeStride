# SafeStride VisualTFT project

이 디렉터리는 `EZ48270M043-LE`용 VisualTFT 3.0 네이티브 소스 프로젝트다.

## 사용 순서

1. VisualTFT에서 `Project.tftprj`를 연다.
2. 모델이 실제 LCD 라벨과 일치하고 해상도가 480 × 272인지 확인한다.
3. 프로젝트를 컴파일한다.
4. 가상 화면에서 인트로, 상태 화면과 영문 출력을 확인한다.
5. 확인이 끝난 뒤 Download Wizard에서 SDCard download를 실행한다.

`output/`, `*_build/`, `SD_PACKET/`, `script.map`은 VisualTFT가 다시 만드는
산출물이므로 Git에 넣지 않는다. SD 카드 패키지는 반드시 현재 프로젝트를 다시
컴파일한 뒤 생성한다.

## 유지보수 규칙

- 화면 1의 기능 컨트롤 ID 1~16과 23은 `../controls.csv` 및 Lua와 연결되므로 바꾸지 않는다.
- ID 17~22는 배경 카드와 상태 바를 위한 장식용 컨트롤이다.
- Modbus 변수 주소와 기본값은 `../registers.csv`가 기준이다.
- 편집 가능한 Lua 원본은 UTF-8인 `../safestride.lua`다.
- VisualTFT 기본 글꼴에서 한글이 `?`로 표시되므로 LCD 문구는 ASCII 영문만 사용한다.
- 교차로 한글 이름은 Pi에서 20바이트 영문 이름으로 변환해 `ss_location_0~9`로 전달한다.
- `main.lua`는 `../safestride.lua`의 프로젝트용 사본이다. Lua를 바꿀 때는 두 파일의
  내용이 같도록 동기화하고 컴파일한다.
- DEV 버튼(ID 11)은 동작과 송신 데이터가 없으며 Lua에서도 비활성화한다.

저장소 테스트는 XML, 컨트롤 ID와 좌표, Modbus 변수 및 두 Lua 사본의 일치를
검사한다. 실제 VisualTFT 컴파일, 화면 렌더링과 LCD 다운로드는 Windows 도구와
실물 화면에서 마지막으로 확인해야 한다.
