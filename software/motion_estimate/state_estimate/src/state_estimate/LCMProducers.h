#ifndef __LCMProducers_h
#define __LCMProducers_h

#include "LCMSubscriber.h"
#include "QueueTypes.h"

// These producers subscribe to LCM messages and add received messages to a
// synchronized queue to be consumed by a separate thread.


namespace StateEstimate
{

template <typename MessageType>
class MessageQueueProducer : public LCMSubscriber
{
public:

  MessageQueueProducer(const std::string& channel) : mChannel(channel)
  {

  }

  virtual void subscribe(boost::shared_ptr<lcm::LCM> lcmHandle)
  {
    lcmHandle->subscribe(this->mChannel, &MessageQueueProducer::messageHandler, this);
  }

  void stop()
  {
    this->mQueue.stopQueue();
  }

  SynchronizedQueue<MessageType>& messageQueue()
  {
    return this->mQueue;
  }

protected:

  void messageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const MessageType* msg)
  {
    VarNotUsed(rbuf);
    VarNotUsed(channel);

    // enqueue a copy of the message
    mQueue.enqueue(*msg);
  }

  std::string mChannel;
  SynchronizedQueue<MessageType> mQueue;
};

typedef MessageQueueProducer<drc::atlas_state_t> AtlasStateMessageProducer;
typedef MessageQueueProducer<drc::robot_state_t> RobotStateMessageProducer;
typedef MessageQueueProducer<drc::atlas_raw_imu_batch_t> IMUBatchMessageProducer;
typedef MessageQueueProducer<bot_core::pose_t> PoseMessageProducer;
typedef MessageQueueProducer<drc::nav_state_t> NavStateMessageProducer;




} // end namespace

#endif // __LCMProducers_h
