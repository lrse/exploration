#ifndef __EXPLORATION_CONFIH_H__
#define __EXPLORATION_CONFIG_H__

#cmakedefine ENABLE_DISPLAY

#cmakedefine ENABLE_SYROTEK

#ifdef ENABLE_SYROTEK
#define PLAYER_SERVER "syrotek"
#define ONLY_ON_SIMULATION(x)
#define POSITION_PROXY_ID 2
#else
#define PLAYER_SERVER "localhost"
#define ONLY_ON_SIMULATION(x) x
#define POSITION_PROXY_ID 0
#endif

#endif
