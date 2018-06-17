#include "Image.h"
#include "Utilities.h"

void FreeImageExample()
{
	Image im(10, 2);

	for (int y = 0; y < 1; y++)
	{
		for (int x = 0; x < 5; x++)
		{
			im[y][x].rgbtBlue = 255;
			RGBTRIPLE a = im[y][x];
			a.rgbtBlue = a.rgbtBlue;
		}
	}

	for (int y = 0; y < 2; y++)
	{
		for (int x = 0; x < 10; x++)
		{
			unsigned r = (unsigned)im[y][x].rgbtRed;
			unsigned g = (unsigned)im[y][x].rgbtGreen;
			unsigned b = (unsigned)im[y][x].rgbtBlue;
			std::cout << " (" << r << "," << g << "," << b << "),";
		}
		std::cout << std::endl;
	}

	im.SavePNG("hello.png");
	system("pause");
}