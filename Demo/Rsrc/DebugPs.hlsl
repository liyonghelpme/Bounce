#include "Engine.h"

void main( in OUT_VECS _ovInVecs, out float4 _vOutColor : SV_TARGET ) {
	_vOutColor = g_euEngineUniforms.vDiffuseMaterial;
}