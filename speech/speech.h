#ifndef __SB_SPEECH_H__
#define __SB_SPEECH_H__

namespace speech {
  int start(void);
  void stop(void);
  char *listen(void);
  void say(const char *fmt);
}

#endif
