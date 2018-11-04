#include "Cube.hlsli"

struct Skin
{
	float4 weight;
	int4 joint_index;
};

StructuredBuffer<Skin> skins : register(t0);

cbuffer joint_color_cb : register(b0)
{
	float4 joint_color[24];
};

struct VertexShaderInput
{
	float3 pos : POSITION;
};

GeometryShaderInput main(VertexShaderInput input, uint id : SV_VertexID)
{
	GeometryShaderInput output = (GeometryShaderInput)0;
	output.pos = float4(input.pos, 1.f);
	float4 color = (float4)0;
	color += skins[id].weight.x * joint_color[skins[id].joint_index.x];
	color += skins[id].weight.y * joint_color[skins[id].joint_index.y];
	color += skins[id].weight.z * joint_color[skins[id].joint_index.z];
	color += skins[id].weight.w * joint_color[skins[id].joint_index.w];
	output.col = color;

	return output;
}