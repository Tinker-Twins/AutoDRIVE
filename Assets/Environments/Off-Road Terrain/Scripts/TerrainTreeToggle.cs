using UnityEngine;

public class TerrainTreeToggle : MonoBehaviour
{
    private void OnEnable()
    {
        var terrains = Object.FindObjectsOfType<Terrain>();
        foreach (Terrain terrain in terrains)
        {
            terrain.drawTreesAndFoliage = false;
        }
    }

    private void OnDisable()
    {
        var terrains = Object.FindObjectsOfType<Terrain>();
        foreach (Terrain terrain in terrains)
        {
            terrain.drawTreesAndFoliage = true;
        }
    }
}
