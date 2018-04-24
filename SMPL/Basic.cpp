#include "Basic.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

std::vector<byte> readShaderFromCSO(const std::string& filename)
{
	std::ifstream file(filename, std::ios::in | std::ios::binary);
	if (!file)
	{
		std::cerr << "[ERROR] Can not open shader file \"" + filename + "\"\n";
	}

	// Stop eating new lines in binary mode!!!
	file.unsetf(std::ios::skipws);

	// get its size:
	std::streampos filesize;

	file.seekg(0, std::ios::end);
	filesize = file.tellg();
	file.seekg(0, std::ios::beg);

	// reserve capacity
	std::vector<byte> v;
	v.reserve((unsigned int)filesize);

	// read the data:
	v.insert(v.begin(),
		std::istream_iterator<byte>(file),
		std::istream_iterator<byte>());

	return v;
}

namespace smpl {

	int printHello() {
		std::cout << "Hello World\n";
		return 0;
	}
}