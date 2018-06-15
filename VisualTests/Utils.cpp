#include "stdafx.h"
#include "Utils.h"

std::vector<byte> readShaderFromCSO(const std::string& filename)
{
	std::ifstream file(filename, std::ios::in | std::ios::binary);
	if (!file)
	{
		MessageBoxA(NULL, (std::string("Could not open ") + filename).c_str(), "File error", MB_ICONERROR | MB_OK);
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