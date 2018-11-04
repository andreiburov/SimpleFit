#include "Cube.hlsli"

cbuffer vertexConstantBuffer : register(b0)
{
	matrix worldView;
	matrix worldViewIT; // Inverse Transpose
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
	for (int i = 0; i < 3; i++)
	{
		PixelShaderInput element = (PixelShaderInput)0;
		float4 posView = mul(worldView, input[i].pos);
		element.posView = posView.xyz;
		float3 normal = calculateNormal(input[0].pos.xyz, input[1].pos.xyz, input[2].pos.xyz);
		element.norView = mul((float3x3)worldViewIT, normal);
		element.pos = mul(projection, posView);
		element.col = input[i].col;
		output.Append(element);
	}
}