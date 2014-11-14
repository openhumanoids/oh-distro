bool gCaughtSignal = false;
Driver gDriver;


void signalHandler(const int signal){
  if (gCaughtSignal) return;
  gCaughtSignal = true;
  std::cout << "Caught signal, stopping driver" << std::endl;
  delete gDriver;
  Channel::Destroy(d);
  exit(1); 
}

int main(const int iArgc, const char** iArgv) {
  return 1;
}
