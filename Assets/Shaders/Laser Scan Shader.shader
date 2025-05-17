Shader "Custom/LaserScanShader"
{
    Properties
    {
        _PointSize("Point Size", Float) = 0.01
        _PointColor("Point Color", Color) = (1, 0, 0, 1)
    }
    SubShader
    {
        LOD 200
        Tags { "RenderType"="Opaque" "ForceNoShadowCasting"="True" }
        Pass {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma geometry geom
            #pragma target 4.0
            #include "UnityCG.cginc"            

            struct vertexIn {
                float4 pos : POSITION;
                float4 color : COLOR;
            };
            struct vertexOut {
                float4 pos : POSITION;
                float4 color : COLOR0;
            };
            struct geomOut {
                float4 pos : POSITION;
                float4 color : COLOR0;
            };

            // Define vertices
            static const float3 _FlatSquareVertices[4] = {
                float3(-1, -1, 0),
                float3(-1,  1, 0),
                float3( 1, -1, 0),
                float3( 1,  1, 0)
            };

            // Define indices
            static const int3 _FlatSquareIndices[2] = {
                int3(0, 1, 3),
                int3(0, 3, 2),
            };

            // Vertex shader
            half4 _PointColor;
            vertexOut vert (vertexIn i)
            {
                vertexOut OUT;
                OUT.pos = i.pos;
                OUT.color = _PointColor; // Set the color for all points
                return OUT;
            }

            // Geometry shader to create points as squares
            float _PointSize;
            [maxvertexcount(6)]
            void geom(point vertexOut IN[1], inout TriangleStream<geomOut> OutputStream)
            {
                geomOut OUT;
                OUT.color = IN[0].color;

                // Render flat squares
                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + mul(float4(_FlatSquareVertices[_FlatSquareIndices[i][j]], 0) * _PointSize / 2., UNITY_MATRIX_V);
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
            }

            // Fragment shader
            float4 frag(geomOut i) : COLOR
            {
                return i.color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}