#include "Std.h"

void Main( in float3 _vInTexCoordDepth : TEXCOORD0, out float4 _vOutColor : SV_TARGET) {
	_vOutColor = float4( 0.0, 0.0, 0.0, 1.0 );

	if (PointTexSpace(_vInTexCoordDepth.xy)) {
		for (int I = -KERNEL_SIZE; I <= KERNEL_SIZE; ++I) {
			float2 vOff = float2(I * TEXEL_SIZE, 0.0);
				_vOutColor.xy += g_t2dDepthMap.Sample(g_ssLinearBorder, _vInTexCoordDepth.xy + vOff).xy;
		}
		_vOutColor.xy /= KERNEL_SIZE * 2.0 + 1.0;
	}
}