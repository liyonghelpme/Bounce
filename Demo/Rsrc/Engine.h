#define MAX_LIGHTS 5

struct IN_VECS {
	float3 vPos : POSITION0;
	float3 vNormal : NORMAL0;
	float3 vBinormal : BINORMAL0;
	float3 vTangent : TANGENT0;
	float3 vBitangent : TANGENT1;
	float2 vTexCoord : TEXCOORD0;
};

struct OUT_VECS {
	float2 vTexCoord : TEXCOORD0;
	float4 vViewNormal : TEXCOORD1;
	float4 vViewPos : TEXCOORD2;
};

struct ENGINE_UNIFORMS {
	float4x4 mWorldView;
	float4x4 mWorldViewProj;
	float4 vAmbientMaterial;
	float4 vBumpMaterial;
	float4 vEmissiveMaterial;
	float4 vDiffuseMaterial;
	float4 vSpecularMaterial;
	float4 vTransparentMaterial;
};

struct LIGHT_FULLDATA {
	unsigned int uiTotalLights;
	unsigned int uiDirLightCount;
	unsigned int uiPointLightCount;
	unsigned int uiSpotLightCount;
	float4 vSpot[MAX_LIGHTS];
	float4 vSpotDir[MAX_LIGHTS];
	float4 vAtt[MAX_LIGHTS];
	float4 vAmbient[MAX_LIGHTS];
	float4 vDiffuse[MAX_LIGHTS];
	float4 vSpecular[MAX_LIGHTS];
	float4 vAmbientLight;
};

cbuffer CB_ENGINE_UNIFORMS : register(cb0) {
	ENGINE_UNIFORMS g_euEngineUniforms;
};

cbuffer CB_LIGHT_FULLDATA : register(cb1) {
	LIGHT_FULLDATA g_lfLightFulldata;
};