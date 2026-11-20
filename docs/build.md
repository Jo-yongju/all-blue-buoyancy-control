# 다른 PC에서 빌드하기

## 준비

- Windows 10/11, Windows PowerShell 5.1 이상
- STM32CubeIDE 1.17.0, 설치에 포함된 GNU Tools for STM32 12.3.rel1
- 이 저장소 전체 ZIP 또는 Git clone

공식 설치 페이지: https://www.st.com/en/development-tools/stm32cubeide.html

ZIP을 전체 압축 해제합니다. `Core` 폴더만 복사하면 프로젝트 설정·시작 코드·링커 스크립트·드라이버가 빠지므로 빌드할 수 없습니다. Python과 별도 MCSDK 설치는 MCU 빌드에 필요하지 않습니다.

## 명령으로 빌드

압축을 푼 저장소 루트에서 PowerShell을 열고 실행합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\Build-Firmware.ps1
```

기본값은 Debug입니다. `C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE` 설치를 먼저 찾습니다. 다른 위치에 설치했다면 `headless-build.bat`이 있는 폴더를 지정합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\Build-Firmware.ps1 -CubeIdePath 'D:\Tools\STM32CubeIDE' -Configuration Release
```

`build/ide-.../` 아래에 새 워크스페이스와 로그를 만들며, 설치 위치를 프로젝트 파일에 저장하지 않습니다. 컴파일러 실행 여부, 빌드 로그, 새 ELF 생성 여부를 확인합니다. 플래시·리셋·모터 구동은 수행하지 않습니다. 기존 CubeIDE에서 같은 프로젝트를 동시에 빌드하지 마세요.

결과: `firmware/updown_dir_control/Debug/updown_dir_control.elf` 또는 `Release/updown_dir_control.elf`.

## IDE에서 빌드

1. 새 워크스페이스로 CubeIDE를 엽니다.
2. File → Import → General → Existing Projects into Workspace에서 `firmware/updown_dir_control`을 선택합니다.
3. 프로젝트 이름 `updown_dir_control`을 확인하고 가져옵니다.
4. Build Configurations → Set Active → Debug, Project → Clean 후 Build Project를 실행합니다.

`.ioc` 코드 재생성은 기존 소스 빌드에 필요하지 않습니다. 본체 F407 이식은 별도 작업이며 이 프로젝트의 MCU는 G431RB입니다.

## 검증 기록

2026-11-20, Windows / CubeIDE 1.17.0 / ARM GCC 12.3.1. 기존 빌드 산출물 없이 새 워크스페이스에서 확인했습니다.

| 구성 | 오류 / 경고 | ELF text / data / bss (bytes) |
|---|---|---|
| Debug | 0 / 0 | 101216 / 608 / 21840 |
| Release | 0 / 0 | 70808 / 476 / 21828 |

이전에 보고된 Windows 오류 623(`Illegal System DLL Relocation`)은 제한 실행 환경에서 컴파일러 프로세스를 시작하지 못한 오류였습니다. 같은 컴파일러와 소스를 일반 실행 환경에서 빌드하자 성공했습니다. 제어 로직 변경은 없으며, 이 결과는 컴파일·링크 확인입니다. 새 보드 다운로드나 실물 모터 시험은 수행하지 않았습니다.
