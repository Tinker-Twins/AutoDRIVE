using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class Toolbar : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    /*
    This script hides the toolbar and makes it visible only
    when hovered in the top region.
    */

    public GameObject toolbar;
    public GameObject MenuPanel;
    public GameObject HUDPanel;

    private bool cursorHover = false;

    void Update()
    {
        if(cursorHover)
        {
            toolbar.SetActive(true);
            // Debug.Log("Show toolbar!");
        }
        else if(MenuPanel.activeSelf || HUDPanel.activeSelf)
        {
            toolbar.SetActive(true);
            // Debug.Log("Keep toolbar visible!");
        }
        else if(!toolbar.activeSelf)
        {
            toolbar.SetActive(false);
            // Debug.Log("Keep toolbar hidden!");
        }
        else
        {
            StartCoroutine(WaitBeforeHiding());
            // Debug.Log("Wait before hiding toolbar!");
        }
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        cursorHover = true;
        // Debug.Log("Cursor enter!");
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        cursorHover = false;
        // Debug.Log("Cursor exit!");
    }

    private IEnumerator WaitBeforeHiding()
    {
        yield return new WaitForSeconds(1.0f); // Wait for 1 seconds
        if(MenuPanel.activeSelf || HUDPanel.activeSelf)
        {
            toolbar.SetActive(true);
            // Debug.Log("Keep toolbar visible!");
        }
        else
        {
            toolbar.SetActive(false);
            // Debug.Log("Hide toolbar!");
        }
    }
}
