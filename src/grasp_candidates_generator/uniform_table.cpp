#include <grasp_candidates_generator/uniform_table.h>


UniformTable::UniformTable(int size)
{
  // Create generator for uniform random numbers.
  boost::mt11213b generator(42u);
  boost::uniform_real<> uni_dist(0.0, 1.0);
  boost::variate_generator<boost::mt11213b&, boost::uniform_real<> > uni(generator, uni_dist);

  table_.resize(size);

  for (int i = 0; i < size; i++)
  {
    table_[i] = uni();
  }
}


double UniformTable::lookup(int idx) const
{
  return table_[idx];
}
