struct LIGHT_ARGS {
	float3 vLightDiffuse;
	float3 vLightSpecular;
	float fDistance;
	float fLdotN;
	float fHdotN;
	float fShininess;
};

void BlinnPhongDir( in LIGHT_ARGS _laLightArgs, inout float3 _vDiffuse, inout float3 _vSpecular ) {
	_vDiffuse += _laLightArgs.vLightDiffuse * _laLightArgs.fLdotN;
	_vSpecular += _laLightArgs.vLightSpecular * pow( _laLightArgs.fHdotN, _laLightArgs.fShininess );
}

void BlinnPhongPoint( in LIGHT_ARGS _laLightArgs, inout float3 _vDiffuse, inout float3 _vSpecular ) {
	_vDiffuse += _laLightArgs.vLightDiffuse * _laLightArgs.fLdotN / _laLightArgs.fDistance;
	_vSpecular += _laLightArgs.vLightSpecular * 
	pow( _laLightArgs.fHdotN, _laLightArgs.fShininess ) / _laLightArgs.fDistance;
}
