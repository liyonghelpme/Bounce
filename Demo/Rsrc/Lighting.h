struct LIGHT_ARGS {
	float3 vLightDiffuse;
	float3 vLightSpecular;
	float fDistance;
	float fLdotN;
	float fHdotN;
	float fShininess;
	float fPower;
};

void BlinnPhongDir( in LIGHT_ARGS _laLightArgs, inout float3 _vDiffuse, inout float3 _vSpecular ) {
	_vDiffuse += _laLightArgs.fLdotN * _laLightArgs.vLightDiffuse;
	_vSpecular += pow(_laLightArgs.fHdotN, _laLightArgs.fShininess) * _laLightArgs.vLightSpecular;
}