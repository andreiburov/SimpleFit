#include "Silhouette.hlsli"

struct VertexShaderInput
{
	float3 pos : POSITION;
};

GeometryShaderInput main(VertexShaderInput input)
{
	GeometryShaderInput output = (GeometryShaderInput)0;
	output.pos = float4(input.pos, 1.f);

	return output;
}