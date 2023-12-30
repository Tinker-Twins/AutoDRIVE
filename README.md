# AutoDRIVE Simulator

![AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE-Simulator.png)

<p align="justify">
AutoDRIVE Simulator is a digital twin of the AutoDRIVE Testbed, which enables the users to virtually prototype their algorithms either due to hardware limitations or as a part of the reiterative development cycle.
</p>

<p align="justify">
The AutoDRIVE Simulator is a pseudo-realistic simulator for scaled autonomous vehicles, which is targeted towards simplicity, modularity and flexibility. The simulator is developed atop the Unity game engine, which enables simulation of realistic physics (using NVIDIA’s PhysX Engine) and photorealistic graphics (using Unity’s Post-Processing Stack). The simulator simulates a comprehensive sensor suite and realistic actuator response, and also features a communication bridge in order to interface externally developed autonomous driving software stack. Presently, the bridge is compatible with Robot Operating System (ROS) and offers a direct scripting support for Python and C++. The bridge supports local as well as distributed computing.
</p>

<p align="justify">
The simulator can be exploited by the users (particularly targeting students and researchers in the field) in order to develop and test their algorithms aimed at autonomous driving.
</p>

## DOWNLOAD AND RUN

1. Download and unzip the [latest release of AutoDRIVE Simulator]((https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.3.0)):
     - [Windows](https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_Windows.zip)
     - [Linux](https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_Linux.zip)
     - [macOS](https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_macOS.zip)

2. Provide adequate permissions to the standalone executable:
     - **Windows:**
         - [Optional] Run as administrator
         - [Optional] Set real-time execution priority
             ```cmd
             > cd <path/to/AutoDRIVE Simulator.exe>
             > start "" /high "AutoDRIVE Simulator.exe"
             ```
     - **Linux:**
         - [Required] Set the execution rights
             ```bash
             $ cd <path/to/AutoDRIVE Simulator.x86_64>
             $ sudo chmod +x AutoDRIVE\ Simulator.x86_64
             ```
     - **macOS:**
         - [Required] Set the execution rights
             ```bash
             $ cd <path/to/AutoDRIVE Simulator.app>
             $ sudo chmod -R +x AutoDRIVE\ Simulator.app/Contents/MacOS
             ```
         - Run the `AutoDRIVE Simulator.app` and [ensure that macOS trusts apps from unidentified developers](https://support.apple.com/guide/mac-help/open-a-mac-app-from-an-unidentified-developer-mh40616/mac)

3. Run the standalone simulator by double-clicking the standalone executable or via command line interface (CLI):
     - **Windows:**
         ```cmd
         > cd <path/to/AutoDRIVE Simulator.exe>
         > start "" "AutoDRIVE Simulator.exe"
         ```
     - **Linux:**
         ```bash
         $ cd <path/to/AutoDRIVE Simulator.x86_64>
         $ ./ AutoDRIVE\ Simulator.x86_64
         ```
     - **macOS:**
         ```bash
         $ cd <path/to/AutoDRIVE Simulator.app>
         $ ./ AutoDRIVE\ Simulator.app
         ```

## INSTALL FROM SOURCE

> ***Note:*** *This section assumes that you have some knowledge and experience working with the Unity game engine. Here is a link to [official Unity tutorials](https://learn.unity.com) to get you started!*

1. [Download](https://unity.com/download) and [install](https://docs.unity3d.com/hub/manual/InstallHub.html) Unity Hub along with Unity 2021.3.9f1 (LTS) or higher.

2. Install AutoDRIVE Simulator (from source):
     
    - Clone the Clone `AutoDRIVE-Simulator` branch of the `AutoDRIVE` repository:
    
      ```bash
      $ git clone --single-branch --branch AutoDRIVE-Simulator https://github.com/Tinker-Twins/AutoDRIVE.git
      ```
    - Unzip source files larger than 100 MB:
      > ***Note:*** *You may delete the `*.zip` and `*.zip.meta` files after the unzipping operation.*
      - [AutoDRIVE/Assets/Environments/Off-Road Terrain/Terrain Meshes/](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes)
        - [Terrain00.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain00.zip)
        - [Terrain01.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain01.zip)
        - [Terrain02.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain02.zip)
        - [Terrain03.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain03.zip)
        - [Terrain04.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain04.zip)
        - [Terrain05.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain05.zip)
        - [Terrain06.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain06.zip)
        - [Terrain07.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain07.zip)
        - [Terrain08.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Off-Road%20Terrain/Terrain%20Meshes/Terrain08.zip)
      - [AutoDRIVE/Assets/Environments/Windridge%20City/Scenes](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator/Assets/Environments/Windridge%20City/Scenes)
        - [Windridge City.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Windridge%20City/Scenes/Windridge%20City.zip)
      - [AutoDRIVE/Assets/Models/Vehicle/RZR](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator/Assets/Models/Vehicle/RZR)
        - [Polaris_RZR.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Models/Vehicle/RZR/Polaris_RZR.zip)
        - [Polaris_RZR_Pro_R_4_Sport.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Models/Vehicle/RZR/Polaris_RZR_Pro_R_4_Sport.zip)
      - [AutoDRIVE/Assets/Scenes](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator/Assets/Scenes)
        - [City.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Scenes/City.zip)
        - [Intersection School.zip](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Scenes/Intersection%20School.zip)

    - Launch Unity Hub and select `ADD` project button. Navigate to the download directory and select the parent folder of the `AutoDRIVE` repository.
  
    - Launch AutoDRIVE Simulator by running the project.
      > ***Note:*** *It may take several minutes to import and load the project for the first time. Please be patient.*
    
    - Bake lightmaps for [Windridge City](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Environments/Windridge%20City/Scenes/Windridge%20City.zip) and [City](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Simulator/Assets/Scenes/City.zip) scenes.
      > ***Note:*** *The lightmap baking process may take several minutes/hours depending upon the computational platform.*

3. Install Machine Learning Toolkit ([Unity ML-Agents](https://unity.com/products/machine-learning-agents))

    - Install ML-Agents Unity Package (tested version: `com.unity.ml-agents v2.0.1`):
     
        The Unity ML-Agents C# SDK is a Unity Package. You can install the `com.unity.ml-agents` package [directly from the Package Manager registry](https://docs.unity3d.com/Manual/upm-ui-install.html). Please make sure to enable 'Preview Packages' in the 'Advanced' dropdown in order to find the latest Preview release of the package.
     
        > ***Note:*** *AutoDRIVE Simulator comes pre-installed with `com.unity.ml-agents v2.0.1`. As such, this step should NOT be necessary. However, in case you face issues importing this Unity package, please consult the [official Unity ML-Agents installation guide](https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Installation.md).*
  
    - Install ML-Agents Python Package (tested version: `mlagents 0.26.0`):
    
      - Create a virtual environment (strongly recommended):
    	    
        ```bash
        $ conda create --name autodrive python=3.8
        ```
          
      - Activate the environment:
      
        ```bash
        $ conda activate autodrive
        ```
    
      - Install `mlagents` package from PyPi (this command also installs the required dependencies including PyTorch):
        
        ```bash
        $ python -m pip install mlagents==0.26.0
        ```
    
        > ***Note:*** *It is strongly recommended that you use packages from the same release together for the best experience. Please consult the [official Unity ML-Agents releases page](https://github.com/Unity-Technologies/ml-agents/releases) for better understanding the version compatibility of different packages.*
