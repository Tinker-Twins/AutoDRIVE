using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class TireFriction : MonoBehaviour
{
    public enum Mode { Default, PhysicsMaterial, FrictionMap };
    public Mode frictionMode = Mode.Default; // Set friction mode
    public float frictionCoefficient = 1.0f; // Friction coefficient [used for `Default` mode]
    public FrictionMap frictionMap; // Friction map  [used for `FrictionMap` mode]
    public bool enableGUI = false; // Display friction value on GUI
    public TextMeshProUGUI frictionText; // Text for friction value
    
    private WheelCollider Tire;
    private float originalSidewaysStiffness;
    private float originalForwardStiffness;
    
    void Start()
    {
        Tire = GetComponent<WheelCollider>();
        originalSidewaysStiffness = Tire.sidewaysFriction.stiffness;
        originalForwardStiffness = Tire.forwardFriction.stiffness;
        if (enableGUI) frictionText.text = "N/A";
    }

    void Update()
    {
        WheelHit hit;
        if (Tire.GetGroundHit(out hit))
        {
            // Debug.Log(hit.point);
            // Debug.Log(hit.force);
            WheelFrictionCurve fFriction = Tire.forwardFriction;
            WheelFrictionCurve sFriction = Tire.sidewaysFriction;
            if (frictionMode == Mode.Default)
            {
                fFriction.stiffness = frictionCoefficient * originalForwardStiffness;
                sFriction.stiffness = frictionCoefficient * originalSidewaysStiffness;
                if (enableGUI) frictionText.text = frictionCoefficient.ToString("F2");
            }
            if (frictionMode == Mode.PhysicsMaterial)
            {
                fFriction.stiffness = hit.collider.material.staticFriction * originalForwardStiffness;
                sFriction.stiffness = hit.collider.material.staticFriction * originalSidewaysStiffness;
                if (enableGUI) frictionText.text = hit.collider.material.staticFriction.ToString("F2");
            }
            if (frictionMode == Mode.FrictionMap)
            {
                fFriction.stiffness = frictionMap.FrictionLookup(hit.point) * originalForwardStiffness;
                sFriction.stiffness = frictionMap.FrictionLookup(hit.point) * originalSidewaysStiffness;
                if (enableGUI) frictionText.text = frictionMap.FrictionLookup(hit.point).ToString("F2");
            }
            Tire.forwardFriction = fFriction;
            Tire.sidewaysFriction = sFriction;
        }
        else {if (enableGUI) frictionText.text = "N/A";}
    }
}
