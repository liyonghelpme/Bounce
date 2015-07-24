#define TEXEL_SIZE 1.0 / 1024.0
#define KERNEL_SIZE 2
#define EXP_C 80.0

/* A POINT TEXTURE-SPACE INTERSECTION FUNCTION. */
bool PointTexSpace( in float2 _vP ) {
	return _vP.x >= 0.0 && _vP.x <= 1.0 && _vP.y >= 0.0 && _vP.y <= 1.0;
}

/* VARIANCE SHADOW MAPPING. */
float linstep(float _fLow, float _fHigh, float _fV) {
	return saturate((_fV - _fLow) / (_fHigh - _fLow));
}

float ShadowLookUpVsm(in float2 _vMoments, float _fComp) {
	/* VARIANCE. */
	float fVar = max(_vMoments.y - (_vMoments.x * _vMoments.x), -0.001);

	/* UPPER BOUND. */
	float fD = _fComp - _vMoments.x;
	float fP = smoothstep(_fComp - 0.02, _fComp, _vMoments.x);
	float fPMax = linstep(0.2, 1.0, fVar / (fVar + (fD * fD)));

	return saturate(max(fP, fPMax));
}

/* EXPONENTIAL SHADOW MAPPING. */
float ShadowLookUpEsm(float _fOcc, float _fRecv) {
	return saturate( exp(-EXP_C * _fRecv) * _fOcc);
}