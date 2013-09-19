#include <iostream>
#include <signal.h>
#include "death_handler.h"
#include "exabot.h"
using namespace std;

bool end = false;

void catch_interrupt(int sig) {
  cout << "Stopping..." << endl;
  end = true;
}

int main(void) {
  Debug::DeathHandler dh;
  signal(SIGINT, catch_interrupt);
  signal(SIGTERM, catch_interrupt);

  HybNav::ExaBot exabot;

  try {
    while(true) {
      exabot.update();
      usleep(0);
      if (end) break;
    }
  }
  catch(const HybNav::ExplorerException& e) {
    if (e.code == HybNav::ExplorerException::END) cout << "Exploration Ended OK" << endl;
    else cout << "Exploration Exception: " << e.what() << endl;
  }

  exabot.stop();
  exabot.update_player();
  exabot.deinitialize();

  return 0;
}

