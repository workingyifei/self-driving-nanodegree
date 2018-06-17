#include <iostream>
#include "mainClasses.cpp"

using namespace std;

struct Players {
    string player1;
    string player2;
} player;
Gameboard game;

Players getUserName();
void promptUserName();
int selectGridPosition(int row, int column);
int check();
void declare(string player, int gameOver);



Players getUserName() {
  string player1, player2;
  cout<<"Enter player1 name:"<<endl;
  cin>>player.player1;
  cout<<"Enter player2 name:"<<endl;
  cin>>player.player2;
  return player;

}

void promptUserName() {
  cout<<"Game Start"<<endl;
  cout<<player.player1<<" vs "<<player.player2<<endl;
}

int selectGridPosition(int row, int column) {
  game.setGameSpace(row, column, 'x');
}

int check() {
  if (game.fourInRow() == 1) {
    cout<<"Four in a row!";
    return 1;
  }

  else if (game.fourInColumn() == 1) {
    cout<<"Four in a column!";
    return 2;
  }


  else if (game.fourInDiagonal() == 1) {
    cout << "Four in a diagonal!";
    return 3;
  }
  else {
    return 0;
  }

}

void declare(string player, int gameOver) {
  switch (gameOver) {
    case 1:
      cout<<"\t"<<player<<" WINs"<<endl;
      break;
    case 2:
      cout<<"\t"<<player<<" WINs"<<endl;
      break;
    case 3:
      cout<<"\t"<<player<<" WINs"<<endl;
      break;
    case 0:
      break;
    }
}


