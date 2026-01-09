# DynamixelSDK Installation Guide

This SDK provides a unified build system for C, C++, and Python libraries, supporting both standalone and ROS 2 environments.

## 1. ROS 2 Environment (Colcon)

Building this package in a ROS 2 workspace installs C/C++ libraries and the Python package.

```bash
cd ~/robotis_ws/src
# Clone the repository if you haven't
# git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

cd ~/robotis_ws
colcon build --symlink-install --packages-select dynamixel_sdk
source install/setup.bash
```

- **Python Usage:**
  ```python
  import dynamixel_sdk
  port_handler = dynamixel_sdk.PortHandler('/dev/ttyUSB0')
  ```

## 2. Standalone Python Environment (Pip)

If you only need the Python library (without C/C++ compilation):

```bash
cd DynamixelSDK/python
pip install .
```

## 3. Standalone C/C++ Environment (CMake)

If you want to install C/C++ libraries system-wide (`/usr/local/lib`):

```bash
cd DynamixelSDK
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

### Force Standalone Build
If you have ROS 2 installed but want to build the SDK without ROS 2 dependencies (Standalone mode), explicitly set the option to OFF:

```bash
cmake .. -DIS_ROS_BUILD=OFF
make -j4
sudo make install
```

## 4. Windows Environment (Visual Studio)

1. Create a `build` directory.
2. Run CMake to generate solution files.

```powershell
mkdir build
cd build
cmake ..
```

3. Open `dynamixel_sdk.sln`, build, and install.
