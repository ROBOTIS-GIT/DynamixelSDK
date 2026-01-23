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

> **Note for Ubuntu 23.04+ / Debian 12+ users:**
> If you encounter an `externally-managed-environment` error, creating a virtual environment is recommended to avoid conflicts with system packages.
>
> **Option 1: Virtual Environment (Recommended)**
> ```bash
> python3 -m venv env_dxl_sdk
> source env_dxl_sdk/bin/activate
> pip install .
> ```
>
> **Option 2: Force System-wide Installation**
> If you consciously want to install it globally:
> ```bash
> pip install . --break-system-packages
> ```

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
   - In the **Solution Explorer**, look for the **CMakePredefinedTargets** folder.
   - Right-click on the **INSTALL** project and select **Build**.
     - This will automatically build the libraries (`ALL_BUILD`) and install them to the system path (e.g., `C:\Program Files (x86)\dynamixel_sdk`).
   - *Note: You may need to run Visual Studio as Administrator to install to system folders.*

## 5. Other Languages (Java, LabVIEW, MATLAB)

These languages use pre-built dynamic libraries (`.so`, `.dll`, `.dylib`) generated from the C library build. Before proceeding, ensure you have built the C library (`libdxl_c` or `dxl_c`) using the steps above (Section 3 or 4).

### Java
1. **Build the C Library**: Follow the "Standalone C/C++ Environment" instructions to build `libdxl_c`.
2. **Setup Eclipse**:
   - Open Eclipse and import the project from `java/protocol_combined` (or specific protocol folder).
   - Right-click project > **Properties** > **Java Build Path** > **Libraries**.
   - Select **Native library location** and point it to your generated `libdxl_c` location (e.g., `DynamixelSDK/build` or installed path).
   - Run the project.

### LabVIEW
1. **Build the C Library**: Ensure `dxl_c.dll` (Windows) or `libdxl_c.so` (Linux) is built.
2. **Library Path**:
   - Open your LabVIEW project from `labview/`.
   - The VIs call the shared library functions. If LabVIEW cannot find the library, a file dialog will prompt you to locate the `dxl_c` library file. Point it to your build output folder.

### MATLAB
1. **Build the C Library**: Ensure the shared library is built.
2. **Setup Paths**:
   - Open MATLAB.
   - Run `matlab/m_basic_function/protocol_combined/read_write/read_write.m` (example).
   - If you encounter load errors, you may need to add the library path:
     ```matlab
     addpath('path/to/DynamixelSDK/c/include/dynamixel_sdk'); % For Header definitions
     loadlibrary('libdxl_c.so', 'dynamixel_sdk.h');
     ```
   - *Note: MATLAB support relies on loading the C library directly. Refer to `loadlibrary` documentation for OS-specific details.*

