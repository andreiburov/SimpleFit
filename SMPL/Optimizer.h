#pragma once

#include "Utils.h"
#include "Definitions.h"
#include "JointProjector.h"
#include "Generator.h"

namespace smpl
{
	class Optimizer
	{
	public:
		Optimizer(const Generator& generate, const JointProjector& project, const std::vector<float>& tracked_joints) :
			generate_(generate), project_(project), tracked_joints_(tracked_joints)
		{
		}

		void operator()(ShapeCoefficients& betas, PoseCoefficients& thetas, float3& scaling, float3& translation)
		{
			Body body = generate_(betas, thetas);
			//body.Dump("new.obj");
			Image image(640, 480);
			body.Draw(image, project_.GetIntrinsics(), scaling, translation);


			std::vector<float> joints = project_(body, scaling, translation);

			/*{
				int w = image.GetWidth();
				int h = image.GetHeight();

				RGBTRIPLE red;
				red.rgbtRed = 255;

				for (uint i = 0; i < COCO_JOINT_COUNT; i++)
				{
					int x = joints[i*2];
					int y = joints[i*2 + 1];

					if ((x >= 0) && (x < w) && (y >= 0) && (y < h))
					{
						image[y][x] = red;
					}
				}
			}*/

			image.SavePNG("projection.png");

			// f(x) = f(a) - f'(a) * (x-a)
			// we will try to do least squares on tracked and estimated joints
			// for this one has to calculate derivative of projected smpl w.r.t. betas and thetas
		}

	private:
		const Generator generate_;
		const JointProjector project_;
		const std::vector<float> tracked_joints_;
	};
}