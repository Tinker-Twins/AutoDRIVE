//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// Gauge: a procedurally generated dashboard gauge


using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class Gauge : MonoBehaviour
	{
	public float startAngle = 45.0f;
	public float endAngle = 315.0f;
	[Range(0, 1)]
	public float originX = 0.5f;
	[Range(0, 1)]
	public float originY = 0.5f;
	[Range(0, 1)]
	public float radiusX = 0.5f;
	[Range(0, 1)]
	public float radiusY = 0.5f;

	[Space(5)]
	public float fullRange = 200.0f;
	public float mainIntervals = 20.0f;
	public int secondaryMarksPerInterval = 1;

	[Space(5)]
	public float mainMarksLen = 8.0f;
	public float mainMarksMargin = -1.0f;
	public float secondaryMarksLen = 8.0f;
	public float secondaryMarksMargin = -1.0f;
	[Space(5)]
	public float labelMargin = 10.0f;
	public float labelValueDivisor = 1.0f;

	[Space(5)]
	public bool drawRedZone = false;
	public float redZoneStart = 160.0f;
	public float redZoneEnd = 200.0f;
	public int redZoneMarksPerInterval = 9;
	public float redMarksLen = 8.0f;
	public float redMarksMargin = -1.0f;

	[Space(5)]
	public Image mainMark;
	public Image secondaryMark;
	public Text markText;
	public Image redZoneMark;


	[ContextMenu("Regenerate")]
	public void Regenerate ()
		{
		RectTransform thisRt = GetComponent<RectTransform>();

		// Remove previous generated elements and create the parent gameobject for the new ones.
		// The new GameObject is moved to the first position in children so it's drawn first.

		ClearGeneratedObjects();

		GameObject go = new GameObject("_Gauge Elements", typeof(RectTransform));
		go.transform.SetParent(transform);
		go.transform.SetSiblingIndex(0);
		go.transform.localScale = Vector3.one;

		// Configure the new RectTransform to match the current one.
		// Pivot and position first; SetInsetAndSizeFromParentEdge use those as base.

		RectTransform rt = go.GetComponent<RectTransform>();
		rt.pivot = Vector2.zero;
		rt.localPosition = Vector3.zero;
		rt.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Left, 0, thisRt.rect.width);
		rt.SetInsetAndSizeFromParentEdge(RectTransform.Edge.Bottom, 0, thisRt.rect.height);

		// Compute metric variables

		float cx = rt.rect.width * originX;
		float cy = rt.rect.height * originY;
		float rx = rt.rect.width * radiusX;
		float ry = rt.rect.height * radiusY;

		int marks = (int)(fullRange / mainIntervals) + 1;
		float deltaAngle = (endAngle-startAngle) / (fullRange / mainIntervals);

		// Generate colored areas first, so in case of collision marks will be drawn over them

		if (drawRedZone && redZoneMark != null)
			{
			float redMarkDeltaAngle = deltaAngle / (redZoneMarksPerInterval + 1);

			for (int i=0; i<marks; i++)
				{
				float mainMarkAngle = startAngle + i * deltaAngle;

				for (int j=1; j<=redZoneMarksPerInterval; j++)
					{
					float redMarkAngle = mainMarkAngle + j * redMarkDeltaAngle;
					float redMarkValue = (redMarkAngle-startAngle) / (endAngle-startAngle) * fullRange;

					if (MathUtility.IsSimilarOrGreater(redMarkValue, redZoneStart) &&
						MathUtility.IsSimilarOrSmaller(redMarkValue, redZoneEnd))
						{
						Image line = InstantiateLine(go, redZoneMark, cx, cy, rx, ry, redMarkAngle, redMarksLen, redMarksMargin);
						line.gameObject.name = "_Gauge RZ " + i.ToString("00") + " " + j.ToString("00");
						}
					}
				}
			}

		// Generate marks and labels per segment

		for (int i=0; i<marks; i++)
			{
			float angle = startAngle + i * deltaAngle;

			// Main mark

			if (mainMark != null)
				{
				Image line = InstantiateLine(go, mainMark, cx, cy, rx, ry, angle, mainMarksLen, mainMarksMargin);
				line.gameObject.name = "_Gauge MM " + i.ToString("00") + " " + (i*mainIntervals).ToString("0.0");
				}

			// Secondary marks

			if (secondaryMark != null)
				{
				float secondaryDeltaAngle = deltaAngle / (secondaryMarksPerInterval + 1);

				for (int j=1; j<=secondaryMarksPerInterval; j++)
					{
					float secondaryAngle = angle + j * secondaryDeltaAngle;
                    float markValue = (secondaryAngle-startAngle) / (endAngle-startAngle) * fullRange;

					// Don't draw secondary marks past the end angle or within red zone

					if (MathUtility.IsSimilarOrSmaller(secondaryAngle, endAngle)
					    && (!drawRedZone || !(MathUtility.IsSimilarOrGreater(markValue, redZoneStart) && MathUtility.IsSimilarOrSmaller(markValue, redZoneEnd)))
						)
						{
						Image line2 = InstantiateLine(go, secondaryMark, cx, cy, rx, ry, secondaryAngle, secondaryMarksLen, secondaryMarksMargin);
						line2.gameObject.name = "_Gauge SM " + i.ToString("00") + "-" + j;
						}
					}
				}

			// Mark label

			if (markText != null)
				{
				Text label = InstantiateText(go, markText, cx, cy, rx, ry, angle);
				label.text = Mathf.RoundToInt((i * mainIntervals) / labelValueDivisor).ToString();
				label.gameObject.name = "_Gauge LB " + i.ToString("00") + " " + label.text;
				}
			}
		}


	Image InstantiateLine (GameObject parent, Image source, float centerX, float centerY, float radiusX, float radiusY, float angle, float length, float margin)
		{
		float x = centerX - (radiusX - margin) * Mathf.Sin(angle * Mathf.Deg2Rad);
		float y = centerY - (radiusY - margin) * Mathf.Cos(angle * Mathf.Deg2Rad);

		Image line = Object.Instantiate(source, parent.transform) as Image;
		line.transform.localPosition = new Vector3(x, y, 0.0f);
		line.transform.localRotation = Quaternion.Euler(0.0f, 0.0f, -angle);
        line.rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, length);

		return line;
		}


	Text InstantiateText (GameObject parent, Text source, float centerX, float centerY, float radiusX, float radiusY, float angle)
		{
		float sinA = Mathf.Sin(angle * Mathf.Deg2Rad);
		float cosA = Mathf.Cos(angle * Mathf.Deg2Rad);

		// Text positions rounded to nearest pixel to prevent blur excess

		float x = Mathf.RoundToInt(centerX - (radiusX - labelMargin) * sinA);
		float y = Mathf.RoundToInt(centerY - (radiusY - labelMargin) * cosA);

		float pivotX = (1.0f-sinA) * 0.5f;
		float pivotY = (1.0f-cosA) * 0.5f;

		Text text = Object.Instantiate(source, parent.transform) as Text;
		text.rectTransform.pivot = new Vector2(pivotX, pivotY);
		text.transform.localPosition = new Vector3(x, y, 0.0f);
		return text;
		}


	public void ClearGeneratedObjects ()
		{
		Transform t = transform.Find("_Gauge Elements");
		if (t != null) DestroyImmediate(t.gameObject);
		}

	}
}
