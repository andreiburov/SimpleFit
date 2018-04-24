#include <fstream>
#include "Body.h"

namespace smpl {

	void Body::dump(const std::string& filename) const
	{
		std::ofstream file(filename, std::ios::out);
		for (auto& v : m_vertices) {
			file << "v " << v.x << " " << v.y << " " << v.z << "\n";
		}
		for (int i = 0; i < m_indices.size(); i++) {
			if (i % 3 == 0) {
				file << "f " << m_indices[i] + 1 << " ";
			} else if (i % 3 == 1) {
				file << m_indices[i] + 1 << " ";
			} else {
				file << m_indices[i] + 1 << "\n";
			}
		}
	}
};