#pragma once

#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include "DXUtils.h"

#ifdef max
#undef max
#undef min
#endif

namespace smpl
{
	const int BETA_COUNT = 10;
	const int THETA_COUNT = 24;
	const int THETA_COMPONENT_COUNT = THETA_COUNT * 3;
	const int THETA_COUNT_WITHOUT_PARENT = THETA_COUNT - 1;
	const int THETA_MATRIX_COMPONENT_COUNT = 207;
	const int VERTEX_COUNT = 6890;
	const int VERTEX_BUFFER_SIZE = VERTEX_COUNT * sizeof(float3);
	const int FACE_COUNT = 13776;
	const int MAX_TRIANGLES_PER_VERTEX = 9;
	const int POSEDIRS_PER_VERTEX = 1426230;	// (6890 * 207) * 3
	const int SHAPEDIRS_PER_VERTEX = 68900;	// (6890 * 10) * 3

	const int HIP_CENTER = 0;
	const int HIP_RIGHT = 1;
	const int HIP_LEFT = 2;
	const int STOMACH = 3;
	const int KNEE_RIGHT = 4;
	const int KNEE_LEFT = 5;
	const int BACKBONE = 6;
	const int ANKLE_RIGHT = 7;
	const int ANKLE_LEFT = 8;
	const int CHEST = 9;
	const int FOOT_RIGHT = 10;
	const int FOOT_LEFT = 11;
	const int SHOULDER_CENTER = 12;
	const int PECK_RIGHT = 13;
	const int PECK_LEFT = 14;
	const int CHIN = 15;
	const int SHOULDER_RIGHT = 16;
	const int SHOULDER_LEFT = 17;
	const int ELBOW_RIGHT = 18;
	const int ELBOW_LEFT = 19;
	const int WRIST_RIGHT = 20;
	const int WRIST_LEFT = 21;
	const int HAND_RIGHT = 22;
	const int HAND_LEFT = 23;
	const int JOINT_COUNT = 24;

	const int PARENT_INDEX[JOINT_COUNT] = {
		-1,	// 0
		HIP_CENTER, // 1
		HIP_CENTER, // 2
		HIP_CENTER, // 3
		HIP_RIGHT, // 4
		HIP_LEFT, // 5
		STOMACH, // 6
		KNEE_RIGHT, // 7
		KNEE_LEFT, // 8
		BACKBONE, // 9
		ANKLE_RIGHT, // 10
		ANKLE_LEFT, // 11
		CHEST, // 12
		CHEST, // 13
		CHEST, // 14
		SHOULDER_CENTER, // 15
		PECK_RIGHT, // 16
		PECK_LEFT, // 17
		SHOULDER_RIGHT, // 18
		SHOULDER_LEFT, // 19
		ELBOW_RIGHT, // 20
		ELBOW_LEFT, // 21
		WRIST_RIGHT, // 22
		WRIST_LEFT // 23
	};

	extern const char* JOINT_FROM_INDEX[JOINT_COUNT];

	const int COCO_NOSE = 0;
	const int COCO_SHOULDER_CENTER = 1;
	const int COCO_SHOULDER_LEFT = 2;
	const int COCO_ELBOW_LEFT = 3;
	const int COCO_WRIST_LEFT = 4;
	const int COCO_SHOULDER_RIGHT = 5;
	const int COCO_ELBOW_RIGHT = 6;
	const int COCO_WRIST_RIGHT = 7;
	const int COCO_HIP_LEFT = 8;
	const int COCO_KNEE_LEFT = 9;
	const int COCO_ANKLE_LEFT = 10;
	const int COCO_HIP_RIGHT = 11;
	const int COCO_KNEE_RIGHT = 12;
	const int COCO_ANKLE_RIGHT = 13;
	const int COCO_EYE_LEFT = 14;
	const int COCO_EYE_RIGHT = 15;
	const int COCO_EAR_LEFT = 16;
	const int COCO_EAR_RIGHT = 17;
	const int COCO_JOINT_COUNT = 18;

	const int COCO_PARENT_INDEX[COCO_JOINT_COUNT] = {
		COCO_SHOULDER_CENTER, // 0
		-1, // 1
		COCO_SHOULDER_CENTER, // 2
		COCO_SHOULDER_LEFT, // 3
		COCO_ELBOW_LEFT, // 4
		COCO_SHOULDER_CENTER, // 5
		COCO_SHOULDER_RIGHT, // 6
		COCO_ELBOW_RIGHT, // 7
		COCO_SHOULDER_CENTER, // 8
		COCO_HIP_LEFT, // 9
		COCO_KNEE_LEFT, // 10
		COCO_SHOULDER_CENTER, // 11
		COCO_HIP_RIGHT, // 12
		COCO_ANKLE_RIGHT, // 13
		COCO_NOSE, // 14
		COCO_NOSE, // 15
		COCO_EYE_LEFT, // 16
		COCO_EYE_RIGHT // 17
	};

	const int RESIDUALS = COCO_JOINT_COUNT * 2 + BETA_COUNT + THETA_COUNT * 3;
	const int UNKNOWNS = 3 + BETA_COUNT + THETA_COUNT * 3;

	struct ShapeCoefficients
	{
		std::vector<float> betas;
		float operator [](int i) const { return betas[i]; }
		float& operator [](int i) { return betas[i]; }

		ShapeCoefficients()
		{
			betas.resize(BETA_COUNT);
		}

		ShapeCoefficients(const ShapeCoefficients& other) :
			betas(other.betas)
		{
		}

		ShapeCoefficients(ShapeCoefficients&& other) :
			betas(other.betas)
		{
		}

		ShapeCoefficients& operator=(const ShapeCoefficients& other)
		{
			auto temp(other);
			betas.swap(temp.betas);
			return *this;
		}


		ShapeCoefficients& operator=(ShapeCoefficients&& other)
		{
			betas.swap(other.betas);
			return *this;
		}

		ShapeCoefficients& operator=(std::initializer_list<float> other)
		{
			uint j = 0;
			for (auto it = other.begin(); j < BETA_COUNT, it != other.end(); ++j, ++it)
			{
				betas[j] = *it;
			}

			return *this;
		}

		void Reset()
		{
			ZeroMemory(betas.data(), BETA_COUNT * sizeof(float));
		}

		friend void operator<<(ShapeCoefficients& shape, std::string& input)
		{
			std::stringstream ss(input);
			for (int i = 0; (i < BETA_COUNT * 3) && !ss.eof(); i++)
			{
				ss >> shape[i];
			}
		}

		friend std::ostream& operator<<(std::ostream& out, const ShapeCoefficients& shape)
		{
			for (uint i = 0; i < BETA_COUNT - 1; i++)
				out << shape[i] << " ";
			out << shape[BETA_COUNT - 1];
			return out;
		}
	};

	constexpr uint ALPHA(int idx)
	{
		return idx * 3;
	}

	constexpr uint BETA(int idx)
	{
		return idx * 3 + 1;
	}

	constexpr uint GAMMA(int idx)
	{
		return idx * 3 + 2;
	}

	struct PoseEulerCoefficients
	{
		std::vector<float3> thetas;
		float3 operator [](int i) const { return thetas[i]; }
		float3& operator [](int i) { return thetas[i]; }

		// access componentwise
		float operator ()(int i) const { return thetas[i / 3].data[i % 3]; }
		float& operator ()(int i) { return thetas[i / 3].data[i % 3]; }

		PoseEulerCoefficients()
		{
			thetas.resize(THETA_COUNT, float3());
		}

		PoseEulerCoefficients(const PoseEulerCoefficients& other) :
			thetas(other.thetas)
		{
		}

		PoseEulerCoefficients(PoseEulerCoefficients&& other) :
			thetas(other.thetas)
		{
		}

		PoseEulerCoefficients& operator=(const PoseEulerCoefficients& other)
		{
			auto temp(other);
			thetas.swap(temp.thetas);
			return *this;
		}

		PoseEulerCoefficients& operator=(PoseEulerCoefficients&& other)
		{
			thetas.swap(other.thetas);
			return *this;
		}

		void Reset()
		{
			ZeroMemory(thetas.data(), THETA_COUNT * sizeof(float3));
		}

		friend void operator<<(PoseEulerCoefficients& pose, std::string& input)
		{
			std::stringstream ss(input);
			for (int i = 0; (i < THETA_COUNT * 3) && !ss.eof(); i++)
			{
				ss >> pose(i);
			}
		}

		friend std::ostream& operator<<(std::ostream& out, const PoseEulerCoefficients& pose)
		{
			for (uint i = 0; i < (THETA_COUNT*3) - 1; i++)
				out << pose(i) << " ";
			out << pose((THETA_COUNT * 3) - 1);
			return out;
		}
	};

	struct PoseAxisAngleCoefficients
	{
		float3 thetas[THETA_COUNT];
		float3 operator [](int i) const { return thetas[i]; }
		float3& operator [](int i) { return thetas[i]; }

		// access componentwise
		float operator ()(int i) const { return thetas[i / 3].data[i % 3]; }
		float& operator ()(int i) { return thetas[i / 3].data[i % 3]; }

		PoseAxisAngleCoefficients()
		{
			ZeroMemory(thetas, THETA_COUNT * sizeof(float) * 3);
		}

		PoseAxisAngleCoefficients(const PoseAxisAngleCoefficients& other)
		{
			memcpy(thetas, other.thetas, THETA_COUNT * sizeof(float) * 3);
		}

		PoseAxisAngleCoefficients& operator=(const PoseAxisAngleCoefficients& other)
		{
			memcpy(thetas, other.thetas, THETA_COUNT * sizeof(float) * 3);
			return *this;
		}

		friend void operator<<(PoseAxisAngleCoefficients& pose, std::string& input)
		{
			std::stringstream ss(input);
			for (int i = 0; (i < THETA_COUNT * 3) && !ss.eof(); i++)
			{
				ss >> pose(i);
			}
		}
	};

	struct Skin
	{
		float4 weight;
		int4 joint_index;
	};

	typedef Eigen::MatrixXf RegressedJoints;

	std::vector<float3> Joints2Vector(const RegressedJoints& joints);

	template <typename T>
	struct Point
	{
		union
		{
			struct { T x; T y; };
			T m[2];
		};

		bool is_defined;

		Point(T x, T y) : x(x), y(y), is_defined(true) {}
		Point() : is_defined(false) {}

		const T& operator[](int i) const
		{
			if (i < 0 && i > 1) throw std::out_of_range("Point has only two components");
			return m[i];
		}

		T& operator[](int i)
		{
			if (i < 0 && i > 1) throw std::out_of_range("Point has only two components");
			return m[i];
		}

		bool operator<(const Point& other) const
		{
			if (x < other.x) return true;
			else if (x == other.x) return y < other.y;
			else return false;
		}

		Point normalized() const
		{
			Point point;
			float norm = sqrt(static_cast<float>(x * x + y * y));
			if (norm < 1e-8f) norm = 1; // for zero norms
			point.x = x / norm;
			point.y = y / norm;
			return point;
		}

		float dot(const Point& other) const
		{
			return static_cast<float>(x*other.x + y * other.y);
		}

		bool IsDefined() const { return is_defined; }
	};
}