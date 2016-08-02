#pragma once

#include <vector>
#include <list>
#include <map>

class MRDLogger
{

protected:
  enum LoggerDataType {
    LOOGER_DATA_TYPE_UNDEF = 0,
    LOGGER_DATA_TYPE_BOOL,
    LOGGER_DATA_TYPE_CHAR,
    LOGGER_DATA_TYPE_INT,
    LOGGER_DATA_TYPE_FLOAT,
    LOGGER_DATA_TYPE_DOUBLE,
    LOGGER_DATA_TYPE_LONG,
    LOGGER_DATA_TYPE_UCHAR,
    LOGGER_DATA_TYPE_UINT,
    LOGGER_DATA_TYPE_ULONG,
    LOGGER_DATA_TYPE_SIZE
  };

  class DataChannel {
  public:
    std::string name;
    std::string unit;
    LoggerDataType type;
    size_t channel;
    const void *ptr;
    std::vector<float> data;

    DataChannel()
    {
      type = LOOGER_DATA_TYPE_UNDEF;
      ptr = NULL;
      channel = 0;
    }
  };

  float _freq;

  size_t _ptEnd;
  size_t _ptStart;

  std::map<const std::string, DataChannel> _channels;
  std::list<const DataChannel *> _outputOrder;

  void reset();

public:
  MRDLogger();
  virtual ~MRDLogger();
  void SaveData();
  void PopData();

  bool ReadFromFile(const std::string &name);
  bool WriteToFile(const std::string &name) const;

  bool AddChannel(const std::string &name, const std::string &unit, const bool *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_BOOL); }
  bool AddChannel(const std::string &name, const std::string &unit, const char *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_CHAR); }
  bool AddChannel(const std::string &name, const std::string &unit, const int *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_INT); }
  bool AddChannel(const std::string &name, const std::string &unit, const float *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_FLOAT); }
  bool AddChannel(const std::string &name, const std::string &unit, const double *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_DOUBLE); }
  bool AddChannel(const std::string &name, const std::string &unit, const long *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_LONG); }
  bool AddChannel(const std::string &name, const std::string &unit, const unsigned char *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_UCHAR); }
  bool AddChannel(const std::string &name, const std::string &unit, const unsigned int *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_UINT); }
  bool AddChannel(const std::string &name, const std::string &unit, const unsigned long *ptr)
    { return AddChannel(name, unit, ptr, LOGGER_DATA_TYPE_ULONG); }

  inline bool has_more_data() const { return size() != 0; }
  inline void set_frequency(float f) { _freq = f; }

  size_t size() const;
  size_t max_size() const;

private:
  bool AddChannel(const std::string &name, const std::string &unit, const void *ptr, LoggerDataType type);
};
