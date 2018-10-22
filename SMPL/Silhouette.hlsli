struct VertexShaderInput
{
	float3 pos : POSITION;
	float3 nor : NORMAL;
};

struct GeometryShaderInput
{
	float4 pos : SV_POSITION;
	float3 nor : NORMAL;
	uint id : VERTEX_ID;
};

struct PixelShaderInput
{
	float4 pos : SV_POSITION;
	float3 pos_view : VIEW_SPACE_POSITION;
	float3 nor_view : VIEW_SPACE_NORMAL;
	float3 barycentric : BARYCENTRIC_COORDINATES;
	uint3 id : VERTEX_INDICES;
};

struct PixelShaderOutput
{
	float4 color;
	float4 normal;
	float4 vertex_indices;
	float4 barycentric;
};