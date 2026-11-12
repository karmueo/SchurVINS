/*
 *  createvoc.cpp
 *
 *  Use this file to create a new vocabulary. Specify the path where the images
 * have been stored in extractFeaturesFromFolder
 *  function. The Vocabulary Name and other associated parameters, have to be
 * specified.
 *
 *  Created on: Nov 8, 2017
 *  Author: kunal71091
 */

#include <fstream>
#include <filesystem>

#include "svo/online_loopclosing/bow.h"

using namespace std;
using namespace svo;
using namespace DBoW2;

int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "Usage: createvoc IMAGE_DIRECTORY VOCABULARY_NAME OUTPUT_DIRECTORY"
              << std::endl;
    return 1;
  }
  string folder_location = std::string(argv[1]);
  // branching factor and depth levels
  const int k = 8;
  const int L = 4;
  const WeightingType weight = TF_IDF;
  const ScoringType score = BHATTACHARYYA;
  const string voc_save_path(argv[3]);
  std::filesystem::create_directories(voc_save_path);

  string voc_name = std::string(argv[2]);
  vector<vector<cv::Mat>> features;
  extractFeaturesFromFolder(folder_location, &features);
  OrbVocabulary voc =
      createVoc(features, voc_save_path, voc_name, k, L, weight, score);

  return 0;
}
