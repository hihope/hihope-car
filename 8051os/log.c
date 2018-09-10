#include "log.h"
 int get_log_len(void)
 {
  int i =0;
  for(i=0;i<MAX_LOG_BUF_LEN;i++)
  {
    if(log_buf[i] == '\n')
    {
      return i+1;
    }
  }
  return MAX_LOG_BUF_LEN;
 } 

