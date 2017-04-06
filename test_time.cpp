
#include <assert.h>
#include <stdio.h>
#define DEBUG_ARTIMETOROSTIME 1
#include "ArTimeToROSTime.h"

int main(int argc, char **argv)
{
  ros::Time::init();

  ros::Time rt = ros::Time::now();
  ArTime art;
  long elapseda = art.mSecSince();
  unsigned long elapsed = ros::Time::now().toNSec() - rt.toNSec();

  for(int i = 0; i < 3; ++i)
  {

    printf("Now:\nArTime sec %ld msec %ld (ns==%ld)\nROS Time sec %f ns %lu (msec==%f)\n",
      art.getSec(), art.getMSec(), art.getMSec() * (long)1e6,
      rt.toSec(), rt.toNSec(), (double) rt.toNSec() / 1e6);
    printf("ArTime Monotonic? %s\n", ArTime::usingMonotonicClock()?"yes":"no");
    ros::Time cart = convertArTimeToROS(art);
    ArTime crt = convertROSTimeToArTime(rt);
    printf("\nConverted:\nArTime from ROS = sec %ld msec %ld (ns==%ld)\nROS Time from ArTime =  sec %f ns %lu (msec==%f)\n",
      crt.getSec(), crt.getMSec(), crt.getMSec() * (long)1e6,
      cart.toSec(), cart.toNSec(), (double) cart.toNSec() / 1e6);
    printf("\nROSTime ConvertedFromArTime-ROSTime Err = %lu ns\n", cart.toNSec() - rt.toNSec());
    printf("ArTime  ConvertedFromROSTime-ArTime Err = %ld ms\n", crt.getMSec() - art.getMSec());
    //printf("Expected err = %lu ns or %ld ms\n", elapseda * (long)1e6, elapseda);
    printf("Expected err = %lu ns or %f ms\n", elapsed, elapsed / 1e6);

    //assert( fabs( cart.toNSec() - rt.toNSec() ) < (elapsed * 2) );
    //assert( fabs( crt.getMSec() - art.getMSec() ) < (elapsed / 1e6) * 2 );

    ArUtil::sleep(500);
    puts("\n----------------------\n");
  }

  return 0;
}
