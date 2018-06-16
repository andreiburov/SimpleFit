#pragma once
#include <SMPL.h>
#include "Camera.h"

class SmplModel
{
public:
	SmplModel(Camera& camera)
		: camera_(camera), generator_(smpl::D3D(), smpl::Generator::Configuration(std::string("Model")))
	{ }

	void Initialize(ID3D11Device* device, ID3D11DeviceContext* device_context);
	void Draw();
	void Clear();

	void Generate(smpl::ShapeCoefficients& shape, smpl::PoseAxisAngleCoefficients& pose);
	void Generate(smpl::ShapeCoefficients& shape, smpl::PoseEulerCoefficients& pose);

private:

	void Generate();
	void UpdateBodyOnGPU();

	ID3D11DeviceContext*	device_context_;
	ID3D11VertexShader*		vertex_shader_ = nullptr;
	ID3D11GeometryShader*	geometry_shader_ = nullptr;
	ID3D11PixelShader*		pixel_shader_ = nullptr;

	ID3D11InputLayout *		input_layout_ = nullptr;
	ID3D11Buffer*			vertex_buffer_ = nullptr;
	ID3D11Buffer*			index_buffer_ = nullptr;
	unsigned int			indices_count_ = 0;

	ID3D11Buffer*			camera_constant_buffer_ = nullptr;

	Camera&					camera_;
	smpl::Body				body_;
	smpl::Generator			generator_;
};