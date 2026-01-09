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

### Selective Build (C vs C++)
By default, both C and C++ libraries are built. You can optimize the build by disabling unused languages using `BUILD_C_LIB` or `BUILD_CXX_LIB` options.

**ROS 2 Example (Build only C++):**
```bash
colcon build --symlink-install --packages-select dynamixel_sdk --cmake-args -DBUILD_C_LIB=OFF
```

**Standalone Example (Build only C):**
```bash
cmake .. -DBUILD_CXX_LIB=OFF
make
sudo make install
```

### Uninstall & Reinstall (Standalone Only)
If you installed the SDK system-wide using `sudo make install`, you can cleanly remove or reinstall it:

```bash
# Remove installed files
sudo make uninstall

# Remove and install again (useful for development)
sudo make reinstall
```

## 4. Windows Environment (Visual Studio)

**Prerequisites:**
* Install [Visual Studio](https://visualstudio.microsoft.com/) 2019 or 2022 (ensure "Desktop development with C++" workload is selected).
* Install [CMake](https://cmake.org/download/). **Note:** During installation, make sure to select **"Add CMake to the system PATH"**.

### Steps

1. Open a Command Prompt (cmd) or PowerShell and navigate to the `DynamixelSDK` folder.
2. Create a `build` directory and run CMake.

```powershell
mkdir build
cd build
cmake ..
```
*Note: This command generates Visual Studio project files (`.vcxproj`) and a solution file (`.sln` or `.slnx`).*

3. **Build & Install:**
   - Open the generated `dynamixel_sdk.sln` (or `dynamixel_sdk.slnx`) file in Visual Studio.
   - In the **Solution Explorer**, looking for the **CMakePredefinedTargets** folder.
   - Right-click on the **INSTALL** project and select **Build**.
     - This will automatically build the libraries (`ALL_BUILD`) and install them to the system path (e.g., `C:\Program Files (x86)\dynamixel_sdk`).
   - *Note: You may need to run Visual Studio as Administrator to install to system folders.*

