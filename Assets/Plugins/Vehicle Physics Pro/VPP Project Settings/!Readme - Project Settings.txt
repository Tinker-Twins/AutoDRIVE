Configuring Project Settings

It is recommended to use Linear color space (Project Settings > Player > Other Settings).

Specific settings files are provided in the folder VPP Project Settings that may be imported
individually. Each file overrides the project’s settings in the corresponding section.


Project Settings - Input (Required)

  Required for some features to work correctly. Alternatively, you could manually
  Configure the input axes Horizontal, Vertical, Fire2 and Fire3 as described here:

  https://vehiclephysics.com/components/vehicle-input/#vpstandardinput


Project Settings - Physics

  Physics settings used in VPP. Note that importing this file overrides your project’s Layer
  Collision Matrix.


Project Settings - Quality

  Enhances the visual quality of the shadows and textures in large scenarios.


Project Settings - Tags And Layers

  VPP uses “User Layer 8” as “Vehicles” for visibility and reflection probes. If you’re
  already using that layer, have in mind that VPP also uses it.

  NOTE - Unity 2019.3: a bug causes an error when importing the Tags And Layers unitypackage.
  You may configure the vehicles layer manually instead:

	1. Go to Project Settings > Tags And Layers > Layers
	2. Name “User Layer 8” as “Vehicles”

