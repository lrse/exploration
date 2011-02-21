#include <iostream>
#include "exabot.h"
using namespace std;

bool end = false;

void catch_interrupt(int sig) {
  cout << "Stopping..." << endl;
  end = true;
}

int main(void) {
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
  catch(...) {
    exabot.stop();
    cout << "Type ctrl+d to end" << endl;
    string a;
    cin >> a;
    exabot.deinitialize();
    throw;
  }

  exabot.deinitialize();

  return 0;
}