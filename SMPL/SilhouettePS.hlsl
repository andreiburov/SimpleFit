#include "Silhouette.hlsli"

float4 main(PixelShaderInput input) : SV_TARGET
{
	/* lambertian illumination
	float3 color = float3(0.9, 0.9, 0.9); // gray
	float3 lightColor = float3(1, 1, 1);

	float3 lightPositionAbove = float3(0, 5, 2); // in view space
	float3 lightPositionBelow = float3(0, -5, 2); // in view space
	float3 lightAbove = normalize(lightPositionAbove - input.posView);
	float3 lightBelow = normalize(lightPositionBelow - input.posView);
	float diffuseAbove = saturate(dot(lightAbove, normalize(input.norView)));
	float diffuseBelow = saturate(dot(lightBelow, normalize(input.norView)));

	float3 light2PositionAbove = float3(0, 5, -6); // in view space
	float3 light2PositionBelow = float3(0, -5, -6); // in view space
	float3 light2Above = normalize(light2PositionAbove - input.posView);
	float3 light2Below = normalize(light2PositionBelow - input.posView);
	float diffuse2Above = saturate(dot(light2Above, normalize(input.norView)));
	float diffuse2Below = saturate(dot(light2Below, normalize(input.norView)));
	
	float4 output = float4(0, 0, 0, 1);
	output.rgb = (diffuseAbove + diffuseBelow + diffuse2Above + diffuse2Below)*0.5*lightColor*color; */

	/* model normals
	float4 output = float4(0, 0, 0, 1);
	output.rgb = input.norView / 2 + 0.5; */

	/* border normals  */
	const float VERTEX_COUNT = 6890.f;
	
	float2 normal = normalize(input.norView.rg) / 2 + 0.5;
	//float4 output = float4(normal.x, normal.y, 1, 1);
	float4 output = float4(input.id[0]/VERTEX_COUNT, input.id[1]/VERTEX_COUNT, input.id[2]/VERTEX_COUNT, 1);

	return output;
}