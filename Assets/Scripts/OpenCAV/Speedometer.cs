using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class Speedometer : MonoBehaviour
{
	public CarController cc;

	private Text digit0, digit1, digit2, gear;

	private float fd0, fd1, fd2;
	private int d0, d1, d2;
	// Use this for initialization
	void Awake ()
	{
		digit0 = transform.GetChild (0).GetComponent<Text> ();
		digit1 = transform.GetChild (1).GetComponent<Text> ();
		digit2 = transform.GetChild (2).GetComponent<Text> ();
		gear = transform.GetChild (3).GetComponent<Text> ();
	}

	// Update is called once per frame
	void Update ()
	{
		fd2 = cc.currSpeed % 10;
		fd1 = (cc.currSpeed / 10) % 10;
		fd0 = (cc.currSpeed / 100) % 10;

		d2 = (int)fd2;
		d1 = (int)fd1;
		d0 = (int)fd0;

		if (d0 < 0)
			d0 *= -1;
		if (d1 < 0)
			d1 *= -1;
		if (d2 < 0)
			d2 *= -1;

		digit2.text = d2.ToString ();
		digit1.text = d1.ToString ();
		digit0.text = d0.ToString ();




		if (cc.gearNum == -1)
			gear.text = "R";
		else if (cc.gearNum == 0)
			gear.text = "N";
		else
			gear.text = cc.gearNum.ToString ();
	}
}
