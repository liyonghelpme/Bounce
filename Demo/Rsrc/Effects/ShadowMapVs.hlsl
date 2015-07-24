#include "Std.h"

void Main( in float3 _vInPos : POSITION0, out float3 _vOutTexCoordDepth : TEXCOORD0, out float4 _vOutPos : SV_POSITION ) {
	_vOutPos = mul( g_euEngineUniforms.mLightWorldViewProj, float4( _vInPos, 1.0 ) );
	_vOutTexCoordDepth = float3( mul( g_euEngineUniforms.mLightWorldViewProjTex, float4( _vInPos, 1.0 ) ).xy, _vOutPos.z / _vOutPos.w );
}