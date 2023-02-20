Shader "Custom/World UV Map"
	{

	Properties
		{
		_Color ("Main Color", Color) = (1,1,1,1)
		_MainTexWall2 ("Wall Side Texture (RGB)", 2D) = "surface" {}
		_MainTexWall ("Wall Front Texture (RGB)", 2D) = "surface" {}
		_MainTexFlr2 ("Flr Texture", 2D) = "surface" {}
		_Scale ("Texture Scale", Float) = 0.1
		}

	SubShader
		{

		Tags { "RenderType"="Opaque" }

		CGPROGRAM
#pragma surface surf Lambert

		struct Input
			{
			float3 worldNormal;
			float3 worldPos;
			};

		sampler2D _MainTexWall;
		sampler2D _MainTexWall2;
		sampler2D _MainTexFlr2;
		float4 _Color;
		float _Scale;

		void surf (Input IN, inout SurfaceOutput o)
			{
			float2 UV;
			fixed4 c;

			if (abs(IN.worldNormal.x)>0.5)
				{
				UV = IN.worldPos.yz; // side
				c = tex2D(_MainTexWall2, UV* _Scale); // use WALLSIDE texture
				}
			else if (abs(IN.worldNormal.z)>0.5)
				{
				UV = IN.worldPos.xy; // front
				c = tex2D(_MainTexWall, UV* _Scale); // use WALL texture
				}
			else
				{
				UV = IN.worldPos.xz; // top
				c = tex2D(_MainTexFlr2, UV* _Scale); // use FLR texture
				}

			o.Albedo = c.rgb * _Color;
			}

		ENDCG
		}

	Fallback "VertexLit"
	}
