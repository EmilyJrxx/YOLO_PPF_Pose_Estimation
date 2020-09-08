#pragma once
#include <cmath>
#include <iostream>
// Helper structs/typedefs to cast buffers to
//
using namespace std;

namespace sen
{
	struct Pixel
	{
		uint8_t Blue;
		uint8_t Green;
		uint8_t Red;
		uint8_t Alpha;
	};

	using DepthPixel = uint16_t;
}