//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// GenericMenu: a toggle-based menu


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using System;


namespace VehiclePhysics.UI
{

public class GenericMenu : MonoBehaviour
	{
	[Serializable]
	public class MenuItem
		{
		public Toggle toggle;
		public GameObject gameObject;
		}

	public MenuItem[] items = new MenuItem[0];

	// Optional: a dropdown that triggers the change.

	public Dropdown dropdown;


	void OnEnable ()
		{
		foreach (MenuItem item in items)
			AddListener(item.toggle, OnValueChanged);

		if (dropdown != null)
			dropdown.onValueChanged.AddListener(OnValueChanged);

		// Re-enabling the object leaves scroll rects when they where.

		UpdateMenuItems(false);
		}


	void OnDisable ()
		{
		foreach (MenuItem item in items)
			RemoveListener(item.toggle, OnValueChanged);

		if (dropdown != null)
			dropdown.onValueChanged.RemoveListener(OnValueChanged);
		}


    void OnValueChanged (bool value)
		{
		// Note: the Toggle that was changed may be read here:
		// EventSystem.current.currentSelectedGameObject

		UpdateMenuItems(true);
		}


    void OnValueChanged (int value)
		{
		UpdateMenuItems(true);
		}


	void UpdateMenuItems (bool updateScroll)
		{
		for (int i = 0; i < items.Length; i++)
			{
			MenuItem item = items[i];

			if (item.gameObject != null)
				{
				bool visible = item.toggle != null && item.toggle.isOn
					|| dropdown != null && dropdown.value == i;

				if (visible && updateScroll)
					{
					ScrollRect[] scrollRects = GetComponentsInChildren<ScrollRect>();

					foreach (ScrollRect sr in scrollRects)
						{
						// Horizontal position is slightly moved to prevent element cropping.
						// Don't reset it here.
						// sr.horizontalNormalizedPosition = 0.0f;
						sr.verticalNormalizedPosition = 1.0f;
						}
					}

				item.gameObject.SetActive(visible);
				}
			}
		}


	void AddListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.AddListener(call);
		}


	void RemoveListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.RemoveListener(call);
		}


	}

}