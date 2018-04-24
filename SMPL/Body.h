#pragma once

#include <DirectXMath.h>
#include <vector>
#include <string>
#include "Definitions.h"

namespace smpl {

	class Body
	{
	public:
		Body() {}

		std::vector<DirectX::XMFLOAT3>& getVertices() { return m_vertices; }
		std::vector<unsigned short>& getIndices() { return m_indices; }
		void dump(const std::string& filename) const;

	private:
		std::vector<DirectX::XMFLOAT3> m_vertices;
		std::vector<unsigned short> m_indices;
	};
};