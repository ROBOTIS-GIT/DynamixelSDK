# SDK Verification

`verification` 폴더는 Dynamixel C++ SDK의 기본 동작을 실제 장비로 확인하기 위한 검증용 실행 파일을 제공합니다.

검증 항목:

- `PortHandler` 생성
- `openPort()`
- `setBaudRate()`
- `ping()` 및 모델별 control table 로딩
- `read1ByteTxRx()`, `read2ByteTxRx()`, `read4ByteTxRx()`
- `broadcastPing()` (Protocol 2.0)
- `GroupSyncRead`, `GroupBulkRead`
- `GroupFastSyncRead`, `GroupFastBulkRead`
- `write1ByteTxRx()` 기반 LED write/read/restore
- `write2ByteTxRx()` 기반 2-byte register write/read/restore
- `write4ByteTxRx()` 기반 4-byte register write/read/restore
- `GroupSyncWrite`, `GroupBulkWrite` 기반 LED write/read/restore
- `GroupSyncWrite`, `GroupBulkWrite` 기반 다축 실제 이동 테스트와 원복
- `Goal Position` 기반 실제 이동 테스트와 원복
- 반복 read loop 테스트
- `closePort()`

## Build

```bash
cmake -S c++ -B build/cpp
cmake --build build/cpp --target install
cmake -S verification -B build/verification
cmake --build build/verification -j
```

`verification`은 시스템에 설치된 `dynamixel_sdk` CMake 패키지를 사용합니다. 기본 prefix가 아닌 위치에 설치했다면 `CMAKE_PREFIX_PATH` 또는 `dynamixel_sdk_DIR`를 지정해서 빌드하면 됩니다.

예시:

```bash
cmake -S verification -B build/verification -DCMAKE_PREFIX_PATH=/usr/local
```

## Run

```bash
./build/verification/dxl_sdk_verifier --device /dev/ttyUSB0 --baud 57600 --protocol 2.0 --ids 1
```

옵션:

- `--device <path>`: 포트 경로
- `--baud <rate>`: baudrate
- `--protocol <1.0|2.0>`: 프로토콜 버전
- `--ids <id[,id...]>`: 검증할 Dynamixel ID 목록
- `--tests <name[,name...]>`: 실행할 테스트 선택
- `--loop-count <count>`: loop 테스트 반복 횟수
- `--move-delta <value>`: move 테스트 목표 위치 오프셋, 기본값 `500`
- `--move-timeout-ms <ms>`: move 테스트 타임아웃
- `--control-table-dir <path>`: control table 폴더 경로
- `--skip-write`: write/sync-write/bulk-write 테스트 생략

기본적으로 `control_table/dynamixel.model`과 각 `.model` 파일을 읽어서 모델별 주소를 자동으로 선택합니다.

예시:

```bash
./build/verification/dxl_sdk_verifier \
  --device /dev/ttyUSB0 \
  --baud 57600 \
  --protocol 2.0 \
  --ids 1,2 \
  --tests all
```

쓰기 테스트는 가능한 경우 아래 순서로 값을 바꾸고 다시 원래 값으로 복구합니다.

- 1-byte: `LED`
- 2-byte: `Goal PWM`, `Moving Speed`, `Max Torque` 중 가능한 항목
- 4-byte: `Goal Velocity`, `Goal Position`, `Homing Offset` 중 가능한 항목

`move` 테스트는 `Goal Position`을 현재 위치에서 작은 오프셋만큼 이동시키고, 완료 후 원래 목표 위치로 복구합니다. 실제 구동이 발생하므로 주변 안전을 확인한 뒤 실행해야 합니다.

각 테스트 단계는 실행 후 `seconds` 단위 소요 시간이 출력되고, 마지막에 `Timing summary`로 단계별/전체 시간이 정리됩니다. 이전 SDK와 비교할 때 같은 장비, 같은 baudrate, 같은 테스트 조합으로 반복 실행하면 됩니다.

각 테스트가 시작될 때도 `[INFO]` 로그로 무엇을 검증하는지 설명이 출력됩니다.

`Present Position`, `Present Velocity`, `Present Current`, `Present Load`처럼 실시간으로 변할 수 있는 필드는 비교 시 기본적으로 `10`의 허용 오차를 둡니다.
