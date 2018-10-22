#pragma once
#include "Definitions.h"
#include "Utils.h"
#include "Image.h"

namespace smpl
{
	class SilhouetteOptimizer
	{
	public:
		SilhouetteOptimizer() {}

		/*Body*/ void operator()()
		{
		}

		void FindCorrespondences(Image& input, Image& model, std::vector<float4>& normals);
	};
}