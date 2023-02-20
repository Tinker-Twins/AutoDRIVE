//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// HoverHandler: shows texts and/or objects on hover


using UnityEngine;
using UnityEngine.UI	;
using UnityEngine.EventSystems;


namespace VehiclePhysics.UI
{

public class HoverHandler : MonoBehaviour,
		IPointerEnterHandler,
		IPointerExitHandler
	{
	[TextArea(5, 10)]
	[UnityEngine.Serialization.FormerlySerializedAs("longDescription")]
	public string hint;

	public Text hintText;
	public GameObject showOnHover;


	void OnEnable ()
		{
		if (showOnHover != null)
			showOnHover.SetActive(false);
		}


	public void OnPointerEnter (PointerEventData eventData)
		{
		if (hintText != null && !string.IsNullOrEmpty(hint))
			hintText.text = hint;

		if (showOnHover != null)
			showOnHover.SetActive(true);
		}


	public void OnPointerExit (PointerEventData eventData)
		{
		if (hintText != null && !string.IsNullOrEmpty(hint))
			hintText.text = "";

		if (showOnHover != null)
			showOnHover.SetActive(false);
		}
	}

}