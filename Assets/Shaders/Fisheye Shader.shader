/*
This shader remaps a wide rectilinear view (a camera's ordinary picture) into an
equidistant fisheye picture: a ray's distance from the picture's centre, in pixels,
is proportional to its angle off the camera's axis. `FisheyeLens` sets its inputs.
*/
Shader "Custom/FisheyeShader"
{
    Properties
    {
        _MainTex ("Wide View", 2D) = "white" {}
    }

    SubShader
    {
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag

            #include "UnityCG.cginc"

            sampler2D _MainTex;
            // x: the fisheye's pixels per radian off the axis; y, z: the picture's width and height in pixels
            float4 _Lens;
            // x, y: the wide view's half width and half height, as tangents of the angle off the axis
            float4 _Wide;

            float4 frag (v2f_img i) : SV_Target
            {
                float2 p = (i.uv - 0.5) * _Lens.yz;
                float r = length(p);
                // the ray's direction on the wide view's image plane, one unit in front of the lens
                float2 t = r > 0 ? tan(r / _Lens.x) * p / r : float2(0, 0);
                return tex2D(_MainTex, 0.5 + 0.5 * t / _Wide.xy);
            }
            ENDCG
        }
    }
}
