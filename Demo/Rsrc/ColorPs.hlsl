#include "Engine.h"
#include "Lighting.h"

void main( in OUT_VECS _ovInVecs, out float4 _vOutColor : SV_TARGET ) {
	float3 vNormalizedViewPosToViewer = normalize( -_ovInVecs.vViewPos.xyz );
	float3 vNormalizedViewNormal = normalize(_ovInVecs.vViewNormal);

	float3 vLightAmbient = float3( 0.0, 0.0, 0.0 );
	float3 vLightDiffuse = float3( 0.0, 0.0, 0.0 );
	float3 vLightSpecular = float3( 0.0, 0.0, 0.0 );

	for ( unsigned int I = 0; I < g_lfLightFulldata.uiPointLightCount; ++I ) {
		//float3 vL = -g_lfLightFulldata.vSpotDir[I].xyz;
		float3 vL = normalize( g_lfLightFulldata.vSpot[I].xyz - _ovInVecs.vViewPos.xyz );

		LIGHT_ARGS laLightArgs;
		laLightArgs.fDistance = length( g_lfLightFulldata.vSpot[I].xyz - _ovInVecs.vViewPos.xyz );
		laLightArgs.vLightDiffuse = g_lfLightFulldata.vDiffuse[I].xyz;
		laLightArgs.vLightSpecular = g_lfLightFulldata.vSpecular[I].xyz;
		laLightArgs.fLdotN = saturate( dot( vL, vNormalizedViewNormal ) );
		laLightArgs.fHdotN = saturate( dot( normalize( vL + vNormalizedViewPosToViewer ), vNormalizedViewNormal ) );
		laLightArgs.fShininess = 25.0f;

		vLightAmbient += g_lfLightFulldata.vAmbient[I].xyz;

		BlinnPhongDir( laLightArgs, vLightDiffuse, vLightSpecular );
	}

	vLightAmbient *= g_euEngineUniforms.vAmbientMaterial.xyz;
	vLightDiffuse *= g_euEngineUniforms.vDiffuseMaterial.xyz;
	vLightSpecular *= g_euEngineUniforms.vSpecularMaterial.xyz;
	
	_vOutColor.xyz = vLightAmbient + vLightDiffuse + vLightSpecular;
	_vOutColor.w = g_euEngineUniforms.vDiffuseMaterial.w;
}