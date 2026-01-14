# DynamixelSDK C++ Examples

This directory contains C++ examples for the Dynamixel SDK.

## Prerequisites

Before building the examples, ensure you have the necessary tools installed.
For detailed installation instructions of the SDK itself, please refer to [INSTALL.md](../../INSTALL.md).

### Linux / macOS
*   **Compiler**: GCC or Clang
*   **Build Tools**: `cmake`, `make`, `sudo` (for install)
    ```bash
    # Ubuntu/Debian
    sudo apt install build-essential cmake
    ```

### Windows
*   **IDE**: [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/) (with "Desktop development with C++" workload)
*   **Build Tools**: [CMake](https://cmake.org/download/) (Add to system PATH during installation)
    
## Build Instructions

The build system has been unified using CMake. You can build these examples in two ways.

## 1. Unified Build (Recommended)
You can build the library and all examples together from the project root. This is the easiest method as it handles dependencies automatically.

```bash
cd /path/to/DynamixelSDK/build
cmake .. -DIS_ROS_BUILD=OFF
make -j
```
*   The executables will be generated in the `build/c++/example` directory (or directly in `build` depending on configuration).

## 2. Standalone Build (Individual)
If you want to build **only** the examples inside this directory, you must first install the Dynamixel SDK library to your system.

### Step 1: Install the Library
```bash
cd /path/to/DynamixelSDK/build
cmake .. -DIS_ROS_BUILD=OFF
make -j
sudo make install
sudo ldconfig
```

### Step 2: Build Examples
```bash
cd /path/to/DynamixelSDK/c++/example
mkdir build && cd build
cmake ..
make
```

### Build Specific Example
You can build a specific example by specifying its target name.
Target names follow the format: `protocol<version>_<function>_cpp`.

```bash
# Example: Build only the Protocol 2.0 Read/Write example
make protocol2_0_read_write_cpp

# Run
./protocol2_0_read_write_cpp
```

## 3. Windows (Visual Studio)

If you are using Visual Studio on Windows, you can generate a solution (`.sln` or `.slnx`) file.

### Prerequisites
*   Ensure the Dynamixel SDK C++ library (`dxl_cpp`) is built and installed (or built within the same solution).
*   Run Visual Studio as **Administrator** if you plan to install libraries to system folders (Program Files).

### Steps
1.  Open Command Prompt or PowerShell.
2.  Navigate to `c++/example/build` (create it if it doesn't exist).
3.  Run CMake to generate the solution:
    ```powershell
    cmake .. -G "Visual Studio 17 2022" -A x64
    ```
4.  Open the generated `dynamixel_sdk_cpp_examples.sln` (or `.slnx`) file in Visual Studio.
5.  In **Solution Explorer**, right-click the example project you want to run (e.g., `protocol2_0_read_write_cpp`).
6.  Select **Set as Startup Project**.
7.  Press **F5** to build and run.
    *   *Note: If the example closes immediately, run via Ctrl+F5 or add a breakpoint/pause at the end.*
