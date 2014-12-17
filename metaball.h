#include "cvec.h"

class Metaball
{
public:
	Cvec3 position;
	double radiusSquared;

	void init(Cvec3 newPosition, double newSquaredRadius)
	{
		position = newPosition;
		radiusSquared = newSquaredRadius;
	}
};