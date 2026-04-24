# ros2_modbus — ROS 2 Modbus TCP Bridge

A pair of ROS 2 packages that expose a Modbus TCP PLC to the ROS 2 graph as
topics and services. The transport is implemented directly on top of POSIX
sockets — **no libmodbus, pymodbus, or any other Modbus library is used**.

| Package          | Lang   | Build type     | What it does                                                      |
| ---------------- | ------ | -------------- | ----------------------------------------------------------------- |
| `ros2_modbus`    | C++17  | `ament_cmake`  | The bridge node + `.srv` definitions. Talks Modbus TCP to the PLC. |
| `ros2_modbus_py` | Python | `ament_python` | A demo client that subscribes to the topics and calls the services. |

---

## Architecture

```mermaid
flowchart LR
    subgraph ROS2["ROS 2 Graph"]
        direction LR
        PY["ros2_modbus_py<br/>modbus_client"]
        BR["ros2_modbus<br/>modbus_bridge"]
    end
    PLC[("PLC<br/>192.168.1.88:502")]

    PY -- "sub /modbus/registers (UInt16MultiArray)" --> BR
    PY -- "sub /modbus/coils (UInt8MultiArray)"     --> BR
    PY -- "srv /modbus/read_registers"   --> BR
    PY -- "srv /modbus/write_register"   --> BR
    PY -- "srv /modbus/read_coils"       --> BR
    PY -- "srv /modbus/write_coil"       --> BR

    BR <-. "Modbus TCP (raw socket)<br/>FC01 / FC03 / FC05 / FC06" .-> PLC
```

### Bridge internal flow

```mermaid
flowchart TD
    START([node start]) --> DECL[declare params:<br/>plc_ip, plc_port, slave_id,<br/>poll_rate_hz, poll_reg_*, poll_coil_*]
    DECL --> REG[create publishers,<br/>services, poll timer]
    REG --> IDLE{event?}

    IDLE -->|timer tick| POLL[read_holding_registers<br/>then read_coils]
    IDLE -->|service call| SVC[dispatch handler:<br/>FC01 / FC03 / FC05 / FC06]

    POLL --> TX[transact under std::mutex]
    SVC  --> TX

    TX --> CONN{socket connected?}
    CONN -- no --> DIAL[connect with 3s<br/>SO_SNDTIMEO/SO_RCVTIMEO]
    DIAL -- fail --> ERR[log error,<br/>return success=false]
    DIAL -- ok --> BUILD
    CONN -- yes --> BUILD["build MBAP header:<br/>tx_id, proto=0, length, unit_id<br/>+ PDU (fc + payload)"]

    BUILD --> SEND[send_all and recv MBAP+PDU]
    SEND --> VAL{"tx_id match?<br/>fc echo?<br/>exception fc OR 0x80?"}
    VAL -- mismatch --> DROP[disconnect socket,<br/>return error]
    VAL -- exception --> EXC[return error,<br/>keep socket]
    VAL -- ok --> DECODE[decode response]

    DECODE -->|poll| PUB[publish UInt16MultiArray<br/>/ UInt8MultiArray]
    DECODE -->|service| RESP[fill response<br/>success + values]
    PUB --> IDLE
    RESP --> IDLE
    ERR --> IDLE
    DROP --> IDLE
    EXC --> IDLE
```

---

## Topics

| Topic                  | Type                         | Direction | Notes                                      |
| ---------------------- | ---------------------------- | --------- | ------------------------------------------ |
| `/modbus/registers`    | `std_msgs/UInt16MultiArray`  | bridge → | Published every poll tick (FC03).          |
| `/modbus/coils`        | `std_msgs/UInt8MultiArray`   | bridge → | One byte per coil (0/1). Published every poll tick (FC01). |

> The original spec asked for `std_msgs/BoolMultiArray`, but no such type
> exists in `std_msgs`. `UInt8MultiArray` is the standard substitute.

## Services

| Service                  | Type                        | Function code |
| ------------------------ | --------------------------- | ------------- |
| `/modbus/read_registers` | `ros2_modbus/ReadRegisters` | FC 03 — Read Holding Registers |
| `/modbus/write_register` | `ros2_modbus/WriteRegister` | FC 06 — Write Single Register  |
| `/modbus/read_coils`     | `ros2_modbus/ReadCoils`     | FC 01 — Read Coils             |
| `/modbus/write_coil`     | `ros2_modbus/WriteCoil`     | FC 05 — Write Single Coil (0xFF00 / 0x0000) |

All services return `bool success` + `string message` and, for reads, the
decoded values.

## Parameters (bridge node)

| Name                 | Type    | Default         |
| -------------------- | ------- | --------------- |
| `plc_ip`             | string  | `192.168.1.88`  |
| `plc_port`           | int     | `502`           |
| `slave_id`           | int     | `0`             |
| `poll_rate_hz`       | double  | `1.0`           |
| `poll_reg_address`   | int     | `0`             |
| `poll_reg_count`     | int     | `10`            |
| `poll_coil_address`  | int     | `0`             |
| `poll_coil_count`    | int     | `8`             |

---

## Layout

```
ros2_ws/
└── src/
    ├── ros2_modbus/               # C++ bridge + srv definitions
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── srv/{ReadRegisters,WriteRegister,ReadCoils,WriteCoil}.srv
    │   ├── include/ros2_modbus/modbus_bridge.hpp
    │   └── src/modbus_bridge.cpp
    └── ros2_modbus_py/            # Python demo client
        ├── package.xml
        ├── setup.py / setup.cfg
        ├── resource/ros2_modbus_py
        └── ros2_modbus_py/modbus_client.py
```

---

## Build (WSL, Ubuntu + ROS 2)

All commands run **inside WSL**. From a Windows shell, prefix with
`wsl bash -c "..."`; from a WSL shell, run them directly.

```bash
# Enter WSL first if you're on PowerShell/cmd:
#   wsl

cd /mnt/c/Users/gnith/Merlin/plc/plc_ws/ros2_ws
source /opt/ros/humble/setup.bash     # or /opt/ros/rolling/setup.bash

colcon build --packages-select ros2_modbus ros2_modbus_py --symlink-install
```

## Run

Terminal 1 — the bridge:

```bash
source install/setup.bash
ros2 run ros2_modbus modbus_bridge
# or with overrides:
ros2 run ros2_modbus modbus_bridge --ros-args \
    -p plc_ip:=192.168.1.88 -p plc_port:=502 \
    -p poll_rate_hz:=2.0 -p poll_reg_count:=20
```

Terminal 2 — the demo Python client (subscribes + calls each service once):

```bash
source install/setup.bash
ros2 run ros2_modbus_py modbus_client
```

## Quick CLI examples

```bash
# Read 10 holding registers starting at 0
ros2 service call /modbus/read_registers \
    ros2_modbus/srv/ReadRegisters "{address: 0, count: 10}"

# Write 1234 to register 0
ros2 service call /modbus/write_register \
    ros2_modbus/srv/WriteRegister "{address: 0, value: 1234}"

# Read 8 coils starting at 0
ros2 service call /modbus/read_coils \
    ros2_modbus/srv/ReadCoils "{address: 0, count: 8}"

# Turn coil 0 on
ros2 service call /modbus/write_coil \
    ros2_modbus/srv/WriteCoil "{address: 0, value: true}"

# Watch the poll topics
ros2 topic echo /modbus/registers
ros2 topic echo /modbus/coils
```

---

## Implementation notes

- **MBAP header** is 7 bytes: `tx_id(2) | proto_id=0 (2) | length(2) | unit_id(1)`.
  `length = 1 + len(PDU)` (unit_id + PDU).
- **Transaction ID** increments per request and wraps at `uint16` max.
  Every response must echo it; a mismatch drops the socket.
- **Exception responses** (`fc | 0x80`) are surfaced as
  `success=false` + `message="modbus exception fc=… code=…"` while the
  TCP connection is kept open.
- **Timeouts** are enforced via `SO_SNDTIMEO` and `SO_RCVTIMEO` (3 s each).
- **Thread safety**: every socket I/O path goes through `transact()` under
  a `std::mutex`, so poll-timer and service callbacks serialize cleanly.
- **Auto-reconnect**: any socket-level failure closes the fd; the next
  `transact()` reopens it. No crash, no backoff loop.
