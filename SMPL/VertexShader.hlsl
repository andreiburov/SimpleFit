struct VertexShaderInput
{
	// skinning
	float4 wgt : WEIGHTS;
	uint4 jdx: JOINT_INDICES;

	float3 pos : POSITION;
	float3 nor : NORMAL;
};

float4 main(VertexShaderInput input) : SV_POSITION
{
	return float4(input.pos, 1.0f);
}