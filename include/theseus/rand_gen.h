#ifndef RANDGEN_H
#define RANDGEN_H

#include <vector>
#include <stdlib.h>
#include <math.h>

namespace theseus
{
class RandGen
{
public:
	RandGen(int seed_in); // Use this contructor - give it a seed
	RandGen();                     // Default contructor - NO SEED. Needed to allow randGen to be a member of a class
	~RandGen();                    // Deconstructor
	double randLin();              // This public function returns a random number from 0 to 1, uniform distribution.
	double norm_rnd(double mu, double sigma);
  std::vector<unsigned int> UINTv(unsigned int len);	// Returns a vector of length len of random unsigned integers
  unsigned int UINT();
private:
	int seed;             // Stores the seed - might be unnecessary.
};
}
#endif
