struct PixelShaderInput
{
	float4 pos : SV_POSITION;
	float3 posView : VIEW_SPACE_POSITION;
	float3 norView : VIEW_SPACE_NORMAL;
	uint3 id : MY_VERTEX_ID;
};

struct GeometryShaderInput
{
	float4 pos : SV_POSITION;
	uint id : VERTEX_ID;
};