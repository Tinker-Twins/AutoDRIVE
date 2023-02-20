How to use the location scenes:

- In Editor:
	Duplicate the scene, then add all other elements (camera, vehicles, scene management, etc)

- By Scripting:
	From a main or loader scene with other elements (camera, vehicles, etc) add the location
	scene via SceneManager.LoadScene or SceneManager.LoadSceneAsync in LoadSceneMode.Additive mode.

The Location scenes do not include cameras.

Location scenes include:

- Scenery
- Lighting
- Ground materials
- Specific components and assets (timing, scripts, shaders...)
- Scene information (spawn points, etc)




