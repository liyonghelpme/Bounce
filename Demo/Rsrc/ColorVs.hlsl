#include "Engine.h"

void main( in IN_VECS _vIn, out OUT_VECS _vOut, out float4 _vOutPos : SV_POSITION ) {
	/* LIGHT SPACE. */
	float4 vLightTexCoord = mul(g_euEngineUniforms.mLightWorldViewProjTex, float4(_vIn.vPos, 1.0));
	_vOut.vLightTexCoordDepth = float3(vLightTexCoord.xy, vLightTexCoord.z / vLightTexCoord.w);

	/* VIEWER SPACE. */
	_vOut.vViewPos = mul( g_euEngineUniforms.mWorldView, float4(_vIn.vPos, 1.0) );
	_vOut.vViewNormal = mul((float3x3)(g_euEngineUniforms.mNormal), _vIn.vNormal);
	_vOut.vTexCoord = _vIn.vTexCoord;

	_vOutPos = mul( g_euEngineUniforms.mProj, _vOut.vViewPos );
}