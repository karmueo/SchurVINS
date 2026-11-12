#include <gtest/gtest.h>
#include <svo/online_loopclosing/bow.h>
#include <stdexcept>

TEST(LoopVocabulary, MissingResourceIsAnExplicitError)
{
  EXPECT_THROW(svo::loadVoc("/nonexistent/schurvins-vocabulary.yml.gz"), std::runtime_error);
}
