#include "AtlasFallDetector.hpp"

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  bool sim_override = false;
  if (argc > 2){
    printf("Usage:\n");
    printf("\tdrc-atlas-fall-detector <sim override = 0>\n");
    exit(0);
  }
  if (argc == 2)
    sim_override = atoi(argv[1]);


  std::shared_ptr<RigidBodyTree> model(new RigidBodyTree(std::string(drc_path) + "/software/models/atlas_v5/model_minimal_contact.urdf"));
  model->compile();

  std::unique_ptr<AtlasFallDetector> fall_detector(new AtlasFallDetector(model, sim_override));
  std::cout << "Atlas fall detector running" << std::endl;
  fall_detector->run();
  return 0;
}

