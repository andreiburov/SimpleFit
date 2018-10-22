#include "Silhouette.hlsli"

PixelShaderOutput main(PixelShaderInput input) : SV_TARGET
{
	PixelShaderOutput output = (PixelShaderOutput)0;
	
	output.color = float4(1.f, 1.f, 1.f, 1.f);
	output.normal = float4(normalize(input.nor_view), 0.f);
	output.vertex_indices = float4(input.id[0], input.id[1], input.id[2], 0.f);
	output.barycentric = float4(input.barycentric, 0.f);

	/* lambertian illumination */
	//float3 color = float3(0.9, 0.9, 0.9); // gray
	//float3 lightColor = float3(1., 1., 1.);

	//float3 lightPositionAbove = float3(0, 5, 2); // in view space
	//float3 lightPositionBelow = float3(0, -5, 2); // in view space
	//float3 lightAbove = normalize(lightPositionAbove - input.pos_view);
	//float3 lightBelow = normalize(lightPositionBelow - input.pos_view);
	//float diffuseAbove = saturate(dot(lightAbove, normalize(input.nor_view)));
	//float diffuseBelow = saturate(dot(lightBelow, normalize(input.nor_view)));

	//float3 light2PositionAbove = float3(0, 5, -6); // in view space
	//float3 light2PositionBelow = float3(0, -5, -6); // in view space
	//float3 light2Above = normalize(light2PositionAbove - input.pos_view);
	//float3 light2Below = normalize(light2PositionBelow - input.pos_view);
	//float diffuse2Above = saturate(dot(light2Above, normalize(input.nor_view)));
	//float diffuse2Below = saturate(dot(light2Below, normalize(input.nor_view)));

	//float3 rgb = saturate((diffuseAbove + diffuseBelow + diffuse2Above + diffuse2Below)*0.5*lightColor*color);
	//output.color = float4(rgb, 1.0f);
	//output.color = float4(normalize(input.nor_view)/2 + 0.5, 1.0f);

	return output;
}