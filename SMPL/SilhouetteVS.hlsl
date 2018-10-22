#include "Silhouette.hlsli"

GeometryShaderInput main(VertexShaderInput input, uint id : SV_VertexID)
{
	GeometryShaderInput output = (GeometryShaderInput)0;
	output.pos = float4(input.pos, 1.f);
	output.nor = input.nor;
	output.id = id;

	return output;
}