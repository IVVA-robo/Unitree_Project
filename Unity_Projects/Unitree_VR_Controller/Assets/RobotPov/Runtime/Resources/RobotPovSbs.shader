Shader "RobotPov/StereoSideBySide"
{
    Properties
    {
        _MainTex ("Side-by-side video", 2D) = "black" {}
        _EyeSwap ("Swap eyes", Float) = 0
        _FlipY ("Flip vertically", Float) = 0
        _Layout ("Video layout: mono=0, SBS=1, top-bottom=2", Float) = 1
        _Signal ("Signal available", Float) = 0
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("Depth test", Float) = 8
    }

    SubShader
    {
        Tags
        {
            "Queue" = "Overlay"
            "RenderType" = "Opaque"
            "IgnoreProjector" = "True"
        }

        Pass
        {
            Cull Off
            ZWrite Off
            ZTest [_ZTest]
            Lighting Off
            Fog { Mode Off }

            CGPROGRAM
            #pragma target 2.0
            #pragma vertex Vert
            #pragma fragment Frag
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            struct AppData
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct VertexToFragment
            {
                float4 position : SV_POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float _EyeSwap;
            float _FlipY;
            float _Layout;
            float _Signal;

            VertexToFragment Vert(AppData input)
            {
                VertexToFragment output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                output.position = UnityObjectToClipPos(input.vertex);
                output.uv = TRANSFORM_TEX(input.uv, _MainTex);
                return output;
            }

            fixed4 Frag(VertexToFragment input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

                if (_Signal < 0.5)
                {
                    return fixed4(0.004, 0.007, 0.012, 1.0);
                }

                float2 uv = input.uv;
                if (_FlipY > 0.5)
                {
                    uv.y = 1.0 - uv.y;
                }

                if (_Layout > 0.5 && _Layout < 1.5)
                {
                    float eye = (float)unity_StereoEyeIndex;
                    if (_EyeSwap > 0.5)
                    {
                        eye = 1.0 - eye;
                    }

                    uv.x = uv.x * 0.5 + eye * 0.5;
                }
                else if (_Layout >= 1.5)
                {
                    float eye = (float)unity_StereoEyeIndex;
                    if (_EyeSwap > 0.5)
                    {
                        eye = 1.0 - eye;
                    }

                    // The left eye occupies the top half of a top-bottom frame.
                    uv.y = uv.y * 0.5 + (eye < 0.5 ? 0.5 : 0.0);
                }

                return tex2D(_MainTex, uv);
            }
            ENDCG
        }
    }

    Fallback Off
}
