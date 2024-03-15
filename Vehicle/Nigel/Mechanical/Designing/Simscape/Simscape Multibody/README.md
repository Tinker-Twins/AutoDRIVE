# Nigel - Simscape™ Multibody™ Model

## Notes

- The STEP (model) files, XML (schema) file, TXT (error) file exported by [Simscape™ Multibody™ Link plugin for SolidWorks®](https://www.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html), MATLAB® script, and Simulink® model of Nigel are compressed (to reduce file size) and available as [`Simscape Multibody.zip`](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Testbed/Vehicle/Mechanical/Designing/Simscape/Simscape%20Multibody/Simscape%20Multibody.zip) within this repository.
- The [`Cylindrical Joint`](https://www.mathworks.com/help/sm/ref/cylindricaljoint.html) exported by [Simscape™ Multibody™ Link plugin for SolidWorks®](https://www.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html) was manually replaced with a [`Revolute Joint`](https://www.mathworks.com/help/sm/ref/revolutejoint.html) for front-right (FR) steering assembly in Simulink® model of Nigel.
- The `Slot Mate` defined between `Steering Horn` and `Steering Pin` in [`Nigel.SLDASM`](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Testbed/Vehicle/Mechanical/Designing/Simscape/SolidWorks%20Assembly/Nigel.SLDASM) was exported as an **unkown constraint** by [Simscape™ Multibody™ Link plugin for SolidWorks®](https://www.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html), which resulted in a **rigid link** in Simulink® model of Nigel. This was replaced manually with a [`Pin Slot Joint`](https://www.mathworks.com/help/sm/ref/pinslotjoint.html) preceeded and suceeded by rigid body transforms to account for different axis orientations in [`Nigel.SLDASM`](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Testbed/Vehicle/Mechanical/Designing/Simscape/SolidWorks%20Assembly/Nigel.SLDASM) (X-Z plane) and requirements of [`Pin Slot Joint`](https://www.mathworks.com/help/sm/ref/pinslotjoint.html) (X-Y plane).
- Directly opening the Simulink® model of Nigel may cause an import error (highlighting certain blocks in red color):
  - This can be simply avoided/resolved by executing `Nigel_DataFile.m` script from [`Simscape Multibody.zip`](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE-Testbed/Vehicle/Mechanical/Designing/Simscape/Simscape%20Multibody/Simscape%20Multibody.zip) in MATLAB®.
  - Alternatively, for a long term fix, open the generated Simulink model (`Nigel.slx`). Navigate to the `Model Workspace` in the `Model Explorer` (located within `DESIGN` pane of `MODELING` tab). Change the `Data Source` of `Workspace Data` from `MATLAB File` to `MATLAB Code`. Copy the contents of the generated `Nigel_DataFile.m` and paste it into the text area. Click Apply.

## References

- [Install the Simscape™ Multibody™ Link Plugin](https://www.mathworks.com/help/smlink/ug/installing-and-linking-simmechanics-link-software.html)
- [Enable Simscape™ Multibody™ Link Plugin in SolidWorks®](https://www.mathworks.com/help/smlink/ref/linking-and-unlinking-simmechanics-link-software-with-solidworks.html)
- [smlink_linksw](https://www.mathworks.com/help/smlink/ref/smlink_linksw.html)
- [smimport](https://www.mathworks.com/help/sm/ref/smimport.html)
- [Mapping SolidWorks® Assembly Mates to Simscape™ Multibody™ Joints and Constraints](https://www.mathworks.com/help/smlink/ref/mates-and-joints.html)
- [Simscape™ Multibody™ Pin Slot Joint](https://www.mathworks.com/help/sm/ref/pinslotjoint.html)
