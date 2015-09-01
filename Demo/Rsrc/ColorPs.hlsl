#include "Engine.h"
#include "Lighting.h"

void main(in OUT_VECS _ovInVecs, out float4 _vOutColor : SV_TARGET) {
	float3 vNormalizedViewPosToViewer = normalize(-_ovInVecs.vViewPos.xyz);
	float3 vNormalizedViewNormal = normalize(_ovInVecs.vViewNormal.xyz);

	float3 vLightAmbient = float3(0.0, 0.0, 0.0);
	float3 vLightDiffuse = float3(0.0, 0.0, 0.0);
	float3 vLightSpecular = float3(0.0, 0.0, 0.0);

	int ui32LightCount = min(g_lfLightFulldata.uiDirLightCount, MAX_LIGHTS);
	for (int I = 0; I < ui32LightCount; ++I) {
		float3 vL = -g_lfLightFulldata.vSpotDir[I].xyz;
		// float3 vL = g_lfLightFulldata.vSpot[I].xyz - _ovInVecs.vViewPos.xyz;
		float lenght = length(vL);
		vL /= lenght;

		LIGHT_ARGS laLightArgs;
		laLightArgs.fDistance = lenght;
		laLightArgs.fPower = 50.0;
		laLightArgs.vLightDiffuse = g_lfLightFulldata.vDiffuse[I].xyz;
		laLightArgs.vLightSpecular = g_lfLightFulldata.vSpecular[I].xyz;
		laLightArgs.fLdotN = saturate(dot(vL, vNormalizedViewNormal));
		laLightArgs.fHdotN = saturate(dot(normalize(vL + vNormalizedViewPosToViewer), vNormalizedViewNormal));
		laLightArgs.fShininess = 0.0;

		vLightAmbient += g_lfLightFulldata.vAmbient[I].xyz;

		BlinnPhongDir(laLightArgs, vLightDiffuse, vLightSpecular);
	}

	vLightAmbient *= g_euEngineUniforms.vAmbientMaterial.xyz;
	vLightDiffuse *= g_euEngineUniforms.vDiffuseMaterial.xyz;
	vLightSpecular *= g_euEngineUniforms.vSpecularMaterial.xyz;

	_vOutColor.xyz = vLightDiffuse;
	_vOutColor.w = g_euEngineUniforms.vDiffuseMaterial.w;
}