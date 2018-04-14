/*	DESCRIPTION:
 *	This is the cpp for the RandGen class. It generates random numbers
 *	by the mersenne twister - make sure to seed the generator.
 *
 */
#include <theseus/rand_gen.h>

namespace theseus
{
RandGen::RandGen(unsigned int seed_in)	// Seed the Random Generator for reproducibility
{
	seed = seed_in;
  srand(seed);
}
RandGen::RandGen()            // This empty function is needed so that RandGen can be a member of a class.
{
}
RandGen::~RandGen()           // Deconstructor
{
}
double RandGen::randLin()     // This public function returns a random number from 0 to 1, uniform distribution
{
	return ((double) rand()/(RAND_MAX));
}
double RandGen::norm_rnd(double mu, double sigma)
{
  double U = randLin();
  double V = randLin();
  return sqrt(-2.0*log(U))*cos(2.0*M_PI*V)*sigma + mu;
}
std::vector<unsigned int> RandGen::UINTv(unsigned int len)
{
	std::vector<unsigned int> uints;			// This function returns a vector of unsigned ints
	for (unsigned int i = 0; i < len; i++)
	{
    double num = ((double) rand()/(RAND_MAX));
		uints.push_back(num);
	}
	return uints;
}
unsigned int RandGen::UINT()
{
	double uints = rand();
	return uints;
}
}
