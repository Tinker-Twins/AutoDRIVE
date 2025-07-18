# Troubleshooting

This directory hosts some troubleshooting tips for the AutoDRIVE Simulink API, which can help debug any build/runtime errors, and assist in installing appropriate dependencies and setting up the right configuration.

## Potential Errors:

Some of the examples require code generation, which may lead to build errors if not installed.

![](No%20Compiler%20Error.png)

Some of the examples require GPU code generation, which may lead to build errors if not installed properly.

![](MEX%20Compiler%20Error.png)

The MEX compiler setup is version-specific and can lead to version-mismatch errors if not installed properly.

![](Visual%20Studio%20Version%20Error.png)

## Potential Fixes:

Download and install the [MATLAB Support for MinGW-w64 C/C++/Fortran Compiler](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-fortran-compiler) support package. This can also be done using MATLAB Add-On Explorer.

Setup the GPU MEX compiler by following the steps below:

1. Install Microsoft Visual Studio with `Desktop development with C++` workload selected [Tested with Visual Studio 2019 (V16.11.49)]

    ![](Visual%20Studio%20Installation.png)

2. Install NVIDIA CUDA [Tested with CUDA 11.8 (V11.8.89)]

    > [!NOTE]
    > It is important to install CUDA after installing Visual Studio so that CUDA can configure Visual Studio settings during installation.

    ![](CUDA%20Installation.png)

3. Setup the MEX Compiler by running `mex -setup C` and `mex -setup C++` in MATLAB Command Window and selecting `Microsoft Visual C++ 2019 (C)` as the compiler.

    ![](MEX%20Compiler%20Setup.png)

4. Set the Simulink build process toolchain to `NVIDIA CUDA (w/Microsoft Visual C++ 2019) | nmake` (selecting `Automatically locate an installed toolchain` should choose this already)

    ![](Build%20Toolchain%20Setup.png)

> [!WARNING]
> - MATLAB/Simulink requires specific versions of Visual Studio and CUDA to be installed. Please refer to the official GPU Coder requirements [here](https://www.mathworks.com/support/requirements/gpu-coder.html).
> - MATLAB/Simulink is known to have trouble working with Visual Studio 2022. Please refer to an official thread about this issue [here](https://www.mathworks.com/matlabcentral/answers/2158705-mexcuda-can-t-find-visual-studio-2022-but-mex-can?s_tid=srchtitle).