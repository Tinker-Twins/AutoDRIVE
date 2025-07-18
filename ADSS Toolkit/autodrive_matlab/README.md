# MATLAB API

<p align="justify">
This directory hosts the MATLAB API for AutoDRIVE, which can be used to develop modular as well as end-to-end autonomous driving algorithms.
</p>

## SETUP

1. Clone `AutoDRIVE-Devkit` branch of the `AutoDRIVE` repository.
    ```bash
    $ git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git
    ```
2. Install the Java library:
   - Place the [`WebSocket-1.0.0.jar`](lib/target/WebSocket-1.0.0.jar) file on the static Java class path in MATLAB by editing the `javaclasspath.txt` file (create the file if it does not exist). Run the following in MATLAB Command Window:
     ```MATLAB
     edit(fullfile(prefdir,'javaclasspath.txt'))
     ```
     For example, if the location of the `jar` file is `C:\AutoDRIVE\ADSS Toolkit\autodrive_matlab\lib\target\WebSocket-1.0.0.jar`, then open the static class path file with the above command and add the full path to it. Make sure that there are no other lines with a `WebSocket-*` entry. You can refer to [MATLAB's Documentation](https://www.mathworks.com/help/matlab/matlab_external/static-path-of-java-class-path.html) for more information on the static Java class path.

     After having done this, **restart MATLAB** and check that the path was read by MATLAB properly by running the `javaclasspath` command. The newly added path should appear at the bottom of the list, before the `DYNAMIC JAVA PATH` entries. Note that seeing the entry here does not mean that MATLAB necessarily found the `jar` file properly. You must make sure that the actual `jar` file is indeed available at this location.
   - **[OPTIONAL]** To build the `jar` file yourself, it is recommended to use [Apache Maven](https://maven.apache.org/download.cgi) (tested with version 3.8.1) with [Java Development Kit](https://www.oracle.com/java/technologies/downloads/?er=221886#java8) (tested with version 8u411). Maven will automatically take care of downloading the [`Java-WebSocket`](https://github.com/TooTallNate/Java-WebSocket) library and neatly package everything into a single file (an "uber jar") based on the [`pom.xml`](lib/pom.xml). Once the `mvn` command is on your path, simply `cd` to the `lib` directory and execute the `mvn package` command.
4. Add the `autodrive_matlab` directory to MATLAB path by right-clicking on it from MATLAB's file explorer and selecting `Add to Path` &rarr; `Selected Folders and Subfolders`.

## USAGE

Execute AutoDRIVE MATLAB API:
```MATLAB
autodrive = example_{vehicle}(4567)
```

Terminate AutoDRIVE MATLAB API:
```MATLAB
autodrive.stop
delete(autodrive)
clear autodrive
```
