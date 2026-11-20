# ALL Blue 부력 제어

수중로봇의 앞뒤 주사기 모듈을 움직이는 STM32G431 펌웨어와 PC 시험 도구입니다. MDD10A로 모터 두 개를 구동하고, 슬라이더 ADC 값으로 각 모듈의 위치를 확인합니다.

2026-11-16 PWM 50% 인계본을 기준으로 정리했습니다. G431 시험용 프로젝트이며 F407 본체 통합본은 아닙니다.

## 구현 내용

- UP / HOVER / DOWN 목표 이동과 목표 도달·통과 시 정지
- 두 모듈 동시 이동, 먼저 도착한 모듈부터 개별 정지
- 수동 JOG, 센서 이상·무진행 감지, 명시적 재시도
- 조종기 SwC 모드 선택과 CH4 피치 목표 입력 연결
- HOVER 중앙 이동 후 IMU 피드백을 사용하는 피치 제어
- FreeRTOS 제어·수신·IMU·콘솔 태스크 분리

위치 이동은 PWM 50% 연속 출력입니다. 피치 제어는 같은 상한 안에서 가변 출력하며, 수동 JOG는 입력한 PWM을 사용합니다. HOVER의 중앙값은 센서 실측 기준이고, 수중 중성부력을 자동으로 찾는 기능은 아닙니다.

## 폴더

| 경로 | 내용 |
|---|---|
| `firmware/updown_dir_control` | CubeIDE 프로젝트, 제어 소스, HAL/CMSIS/FreeRTOS |
| `scripts` | 모터 선택·위치 이동·동시 이동·수동 JOG |
| `tests` | 하드웨어 모사 기반 제어 회귀 시험 |
| `examples` | 본체 수신기·IMU 코드 연결 예제 |
| `docs` | 핀 연결, 태스크 구조, 통합 시 확인 사항 |

## 실행

새 PC에서는 STM32CubeIDE 1.17.0을 설치하고 저장소 ZIP을 **전체 압축 해제**합니다. 필요한 HAL/CMSIS/FreeRTOS 소스가 포함되어 있어 별도로 생성할 필요가 없습니다.

저장소 루트에서 아래 명령으로 새 워크스페이스를 만들고 클린 빌드할 수 있습니다. 보드 연결은 필요하지 않으며 플래시는 수행하지 않습니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\Build-Firmware.ps1
```

설치 경로 지정, Release 빌드와 검증 결과는 [빌드 안내](docs/build.md)에 정리했습니다.

1. STM32CubeIDE 1.17의 Existing Projects into Workspace로 `firmware/updown_dir_control`을 가져옵니다.
2. Debug 구성을 빌드한 뒤 NUCLEO-G431RB에 다운로드합니다. 디버거가 정지해 있으면 Resume합니다.
3. 배선은 [연결 문서](docs/wiring.md)를 확인합니다. ST-LINK Virtual COM 포트는 115200 / 8N1입니다.
4. 저장소 루트의 PowerShell에서 실행합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Position.ps1
```

모터 번호와 COM 포트를 선택하고 HOVER / UP / DOWN을 입력합니다. 이동 중 S는 정지, Q는 메뉴 복귀 또는 종료입니다. 펌웨어 응답의 `CONTINUOUS50`을 확인합니다.

호스트 시험은 Python 3와 PATH에 등록된 GCC가 필요합니다.

```text
python tests/run_tests.py
```

## 확인 범위

개별 모듈의 정·역회전과 목표 이동 시험 이력이 있습니다. 이후 기어·기어박스 마모로 동작 재현성이 떨어졌습니다. PWM50의 최종 기구 부하 시험, F407 이식, 실제 조종기·IMU 연동과 수중 응답은 추가 확인이 필요합니다. `live_config.h`의 LIVE 출력은 기본 비활성입니다.

호스트 시험은 모터·센서를 모사합니다. 실물 내구성이나 수중 운전을 검증하는 시험은 아닙니다.

외부 라이브러리의 저작권과 라이선스는 [THIRD_PARTY.md](THIRD_PARTY.md)를 참고하세요.
