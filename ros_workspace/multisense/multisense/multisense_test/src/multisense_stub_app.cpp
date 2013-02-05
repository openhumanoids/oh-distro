

#include <multisense_test/multisense_stub.h>

#include <arpa/inet.h> // For inet_addr

using namespace multisense_test;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multisensor_stub");

  int dest_addr = inet_addr("127.0.0.1");
  int dest_port = 10000;

  multisense_test::MultisenseStub stub(dest_addr, dest_port, 9000);

  ros::spin();

  return 0;
}
