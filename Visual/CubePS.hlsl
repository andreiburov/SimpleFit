#include "Cube.hlsli"

float4 main(PixelShaderInput input) : SV_TARGET
{
	float3 color = float3(0.9, 0.9, 0.9); // gray
	float3 lightColor = float3(1, 1, 1);

	// for cube
	//float3 lightPositionAbove = float3(0, 2, -5); // in view space
	//float3 lightPositionBelow = float3(0, -2, -5); // in view space

	// for smpl
	float3 lightPositionAbove = float3(0, 5, 2); // in view space
	float3 lightPositionBelow = float3(0, -5, 2); // in view space

	float3 lightAbove = normalize(lightPositionAbove - input.posView);
	float3 lightBelow = normalize(lightPositionBelow - input.posView);
	float diffuseAbove = saturate(dot(lightAbove, normalize(input.norView)));
	float diffuseBelow = saturate(dot(lightBelow, normalize(input.norView)));

	float4 output = float4(0, 0, 0, 1);
	output.rgb = (diffuseAbove + diffuseBelow)*0.5*lightColor*color;

	return output;
}