#include "Silhouette.hlsli"

cbuffer vertexConstantBuffer : register(b0)
{
	matrix world_view;
	matrix world_view_it; // Inverse Transpose
	matrix projection;
};

float3 calculateNormal(float3 v0, float3 v1, float3 v2)
{
	float3 u = v0 - v1;
	float3 v = v1 - v2;
	float3 n = normalize(cross(u, v));

	return n;
}

[maxvertexcount(3)]
void main(
	triangle GeometryShaderInput input[3] : SV_POSITION,
	inout TriangleStream< PixelShaderInput > output
)
{
	//float3 normal = calculateNormal(input[0].pos.xyz, input[1].pos.xyz, input[2].pos.xyz);

	[unroll]
	for (int i = 0; i < 3; i++)
	{
		PixelShaderInput element = (PixelShaderInput)0;

		float4 pos_view = mul(world_view, input[i].pos);
		element.pos = mul(projection, pos_view);
		element.pos_view = pos_view.xyz;
		element.nor_view = mul((float3x3)world_view_it, input[i].nor);
		element.barycentric[i] = 1.0f;

		element.id[0] = input[0].id;
		element.id[1] = input[1].id;
		element.id[2] = input[2].id;

		output.Append(element);
	}
}