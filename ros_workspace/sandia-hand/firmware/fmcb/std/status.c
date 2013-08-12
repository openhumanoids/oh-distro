#include "status.h"

status_t g_status;

void status_init()
{
  for (int i = 0; i < STATUS_PAYLOAD_LEN; i++)
    *(((uint8_t *)&g_status) + i) = 0;
}

