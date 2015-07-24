#include "Std.h"

void Warp(float _fVal, inout float2 _vRet) {
	_fVal = (_fVal * 2.0) - 1.0;
	_vRet.x = exp(80.0 * _fVal);
	_vRet.y = -exp(80.0 * _fVal);
}

void Main( in float3 _vInTexCoordDepth : TEXCOORD0, out float4 _vOutColor : SV_TARGET ) {
	/* EXPONENTIAL SHADOW MAPPING. */
	//_vOutColor = float4( exp( EXP_C * _vInTexCoordDepth.z), 1.0, 1.0, 1.0);

	/* VARIANCE SHADOW MAPPING. */
	float fDx = ddx(_vInTexCoordDepth.z);
	float fDy = ddy(_vInTexCoordDepth.z);
	_vOutColor = float4(_vInTexCoordDepth.z, (_vInTexCoordDepth.z * _vInTexCoordDepth.z) + 0.25 * (fDx * fDx + fDy * fDy), _vInTexCoordDepth.z, 1.0);

	/* EXPONENTIAL VARIANCE SHADOW MAPPING. */
	//ExpWarp( _fInDepth, _vOutColor.xy );
	//_vOutColor.zw = _vOutColor.xy * _vOutColor.xy;
}