<!-- SPDX-FileCopyrightText: 2025 FANUC America Corp.
     SPDX-FileCopyrightText: 2025 FANUC CORPORATION

     SPDX-License-Identifier: Apache-2.0
-->
<!-- markdownlint-disable MD013 -->
# fanuc_driver

![FANUC ROS 2 Control Driver](/images/FANUC_ros2_ControlDriver.jpg "FANUC ROS 2 Control Driver")

## About

This repository hosts the source code of the FANUC ROS 2 Driver project, a ros2_control high-bandwidth streaming driver.
This project will allow users to develop a ROS 2 application to control a FANUC virtual or real robot.

## Installation

See the [FANUC ROS 2 Driver Documentation](https://fanuc-corporation.github.io/fanuc_driver_doc/) for instructions.

## Licensing

The original FANUC ROS 2 Driver source code and associated documentation
including these web pages are Copyright (C) 2025 FANUC America Corporation
and FANUC CORPORATION.

Any modifications or additions to source code or documentation
contributed to this project are Copyright (C) the contributor,
and should be noted as such in the comments section of the modified file(s).

FANUC ROS 2 Driver is licensed under
     [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

Exceptions:

- The sockpp library is licensed under the terms of the [BSD 3-Clause License](https://opensource.org/license/BSD-3-Clause).

- The readwriterqueue library is licensed under the terms of
  the [Simplified BSD License](https://opensource.org/license/BSD-2-Clause).

- The reflect-cpp and yaml-cpp libraries are licensed under the
  terms of the [MIT License](https://opensource.org/license/mit).

Please see the LICENSE folder in the root directory for the full texts of these licenses.



### Manual CMake build & install (libraries only)

> 이 절차는 ROS 2/colcon 없이 **코어 C++ 라이브러리(`fanuc_libs`)**만 빌드·설치하고,
> 외부 CMake 프로젝트에서 `find_package(fanuc_libs)`로 사용하는 방법입니다.
> 전체 드라이버(ROS 2 통합)를 원하시면 위의 문서(Quick Start)를 참고하세요.

#### 0) Prerequisites
```bash
sudo apt update
sudo apt install -y git git-lfs cmake g++ build-essential
git lfs install
# (환경에 따라 필요할 수 있음)
# sudo apt install -y libfmt-dev
```

#### 1) Get sources
```bash
git clone --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git
cd fanuc_driver/fanuc_libs
```
> 이 디렉터리는 내부 `dependencies/`를 **FetchContent**로 사용하도록 구성되어 있습니다.  
> `--recurse-submodules`로 종속 저장소가 함께 내려와야 합니다.

#### 2) Configure & build
```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=0 \
  -DBUILD_EXAMPLES=0
cmake --build build -j
```

#### 3) Install
```bash
# 기본: /usr/local 로 설치
sudo cmake --install build

# (선택) 커스텀 경로로 설치
# sudo cmake --install build --prefix /opt/fanuc
```

설치가 끝나면 CMake 패키지 파일이 아래 위치에 생성됩니다.
- `/usr/local/lib/cmake/fanuc_libs/` (또는 지정한 prefix 경로)

#### 4) Use in your CMake project
소비자 프로젝트의 `CMakeLists.txt` 예시:
```cmake
cmake_minimum_required(VERSION 3.22)
project(consumer_app LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(fanuc_libs CONFIG REQUIRED)

add_executable(demo main.cpp)
target_link_libraries(demo PRIVATE
  fanuc_libs::rmi
  fanuc_libs::stream_motion
  # fanuc_libs::fanuc_client
  # fanuc_libs::gpio_config
)
```

빌드 명령:
```bash
# 기본 경로에 설치했다면 보통 자동 인식됩니다.
cmake -S . -B build

# 커스텀 prefix로 설치했으면 prefix를 알려주세요.
# cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/fanuc
cmake --build build
```

#### Troubleshooting
- **`fatal error: fmt/format.h: No such file or directory`**
  - `sudo apt install -y libfmt-dev` 설치 후, 소비자 프로젝트에서:
    ```cmake
    find_package(fmt REQUIRED)
    target_link_libraries(demo PRIVATE fmt::fmt) # 또는 fmt::fmt-header-only
    ```
- **정적 라이브러리 PIC 에러 (`recompile with -fPIC`)**
  - 깨끗한 빌드(기존 `build/` 제거) 후 재빌드하세요. `fanuc_libs`는 `POSITION_INDEPENDENT_CODE`를 활성화합니다.
  - 외부 정적 라이브러리를 함께 링크한다면, 해당 라이브러리도 `-fPIC`(또는 `CMAKE_POSITION_INDEPENDENT_CODE=ON`)로 빌드해야 합니다.
- **`find_package(fanuc_libs)`를 못 찾는 경우**
  - 설치 경로를 CMake에 알려주세요:
    ```bash
    cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local
    # 또는 커스텀 경로
    # cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/fanuc
    ```
