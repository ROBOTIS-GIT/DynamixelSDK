# Dynamixel SDK C++ Examples

This directory contains **C++ example source codes** for the Dynamixel SDK.

## Prerequisites

Before building the examples, ensure you have the necessary tools installed.
For detailed installation instructions of the SDK library itself, please refer to [INSTALL.md](../../INSTALL.md).

### Linux / macOS
* **Compiler**: GCC or Clang
* **Build Tools**: `cmake`, `make`, `sudo` (for install)
    ```bash
    # Ubuntu/Debian
    sudo apt update
    sudo apt install build-essential cmake
    ```

### Windows
* **IDE**: [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/) (with "Desktop development with C++" workload)
* **Build Tools**: [CMake](https://cmake.org/download/) (Add to system PATH during installation)

---

## Build Instructions (Linux / macOS)

> **Note:** The example binaries are **NOT** built by default when building the SDK root. You must build them separately using the steps below.

### Step 1: Install the SDK Library (Required)
The examples depend on the Dynamixel SDK C++ library (`libdxl_cpp`). You must build and install it first.

```bash
# Go to the SDK root directory
cd /path/to/DynamixelSDK

# Build and Install the library
mkdir build && cd build
cmake .. -DIS_ROS_BUILD=OFF
make -j
sudo make install
sudo ldconfig
```

### Step 2: Build C++ Examples
Once the library is installed, navigate to the example directory to build the examples.

```bash
# Go to the C++ example directory
cd /path/to/DynamixelSDK/c++/example

# Build examples
mkdir build && cd build
cmake ..
make
```

### Build Targets vs. Output Filenames
To avoid conflicts in unified builds, the CMake target names for C++ examples end with `_cpp`, but the actual generated executable filenames are clean (without `_cpp`).

| Make Target Name (Build) | Output Executable Name (Run) | Description |
| :--- | :--- | :--- |
| `read_write_protocol2_cpp` | `read_write_protocol2` | Protocol 2.0 Read/Write |
| `ping_protocol2_cpp` | `ping_protocol2` | Protocol 2.0 Ping |
| `scan_dynamixel_cpp` | `scan_dynamixel` | Network Scan |
| ... | ... | ... |

**Example: Building a specific example only**
```bash
# 1. Build using the target name (with _cpp)
make read_write_protocol2_cpp

# 2. Run the executable (without _cpp)
./read_write_protocol2
```

---

## Build Instructions (Windows)

If you are using Visual Studio on Windows, follow these steps to generate a solution file.

1.  **Open Command Prompt or PowerShell**.
2.  Navigate to the build directory:
    ```powershell
    cd \path\to\DynamixelSDK\c++\example
    mkdir build
    cd build
    ```
3.  **Generate the Solution**:
    ```powershell
    cmake .. -G "Visual Studio 17 2022" -A x64
    ```
4.  Open the generated `dynamixel_sdk_cpp_examples.sln` (or `.slnx`) file in Visual Studio.
5.  In **Solution Explorer**, right-click the example project you want to run (e.g., `read_write_protocol2_cpp`).
6.  Select **Set as Startup Project**.
7.  Press **F5** to build and run.

> **Tip:** If the console window closes immediately after execution, run the project using **Ctrl + F5** (Start Without Debugging).

---

## Execution & Troubleshooting

### 1. Library Not Found Error
If you encounter an error like `error while loading shared libraries: libdxl_cpp.so`, ensure the library path is known to the system.
* **Linux**: Run `sudo ldconfig` after installing the library.
* Check if `/usr/local/lib` is in your `LD_LIBRARY_PATH`.

### 2. Permission Denied (USB Port)
Accessing the USB serial port usually requires specific permissions.
* **Temporary fix**:
    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```
* **Permanent fix**: Add your user to the `dialout` group (Linux).
    ```bash
    sudo usermod -aG dialout $USER
    # (You must logout and login again for this to take effect)
    ```

---

## References
* For detailed SDK installation: [INSTALL.md](../../INSTALL.md)
* For Source Code: See the `c++/example` directory structure.