#if 0

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#else
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

int main(int argc, char** argv)
{
	int mArgc = 2;
	const char* mArgv[2] = { argv[0], "\"PR2D From File\"" };
	int result = Catch::Session().run(mArgc, const_cast<char**>(mArgv));
	system("pause");
    return 0;
}
#endif