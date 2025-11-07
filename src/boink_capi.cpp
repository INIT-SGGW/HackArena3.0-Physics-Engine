#include "boink_capi.h"
#include "boink.h"

BoinkHandle create_engine()
{
  BoinkEngine *engine=new BoinkEngine();

  return (BoinkHandle)engine;
}

void destroy_engine(BoinkHandle handle)
{
  BoinkEngine *engine=(BoinkEngine*)handle;
  if(engine!=nullptr)
  {
    delete engine;
  }
}
