#include "Engine.h"

void main( in IN_VECS _vIn, out OUT_VECS _vOut, out float4 _vOutPos : SV_POSITION ) {
	// Assuming Uniform Scaled World Matrix
	_vOut.vViewPos = mul(g_euEngineUniforms.mWorldView, float4(_vIn.vPos, 1.0));
	_vOut.vViewNormal = mul(g_euEngineUniforms.mWorldView, float4(_vIn.vNormal, 0.0));
	_vOut.vTexCoord = _vIn.vTexCoord;
	_vOutPos = mul(g_euEngineUniforms.mWorldViewProj, float4(_vIn.vPos, 1.0));
}