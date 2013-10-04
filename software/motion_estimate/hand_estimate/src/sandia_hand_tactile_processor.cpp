
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>



using namespace std;

class App{
public:
  App();
  ~App();

private:

  lcm::LCM lcm_ ;
};
App::App(){}
int main(int argc, char **argv){
  
}
